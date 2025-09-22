
#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>
#include "ADXL345.h"


#ifndef ADXL345_TYPES_DEFINED
#define ADXL345_TYPES_DEFINED
typedef uint8_t adresse_type;   // adresse d'un registre ADXL345 (6 bits utiles)
typedef uint8_t val_type;       // valeur écrite/lu sur 8 bits
typedef uint8_t type_retour;    // valeur de retour (lecture registre)
#endif

/* -------------------------------------------------------------------------- */
/* Helpers de sélection de puce SPI (Chip Select)                             */
/* SELECT  = 0  -> nCS = 0 (actif)                                            */
/* RELEASE = 1  -> nCS = 1 (inactif)                                         */
/* -------------------------------------------------------------------------- */
#ifndef SELECT
#define SELECT 0U
#endif
#ifndef RELEASE
#define RELEASE 1U
#endif

/* Chip Select de l'ADXL345 connecté sur PA4 (BSRR écrit 1 pour set/reset) */
static void CS_ADXL(uint32_t state)
{
    if (state == SELECT) {
        /* BSRR[31:16] = bits de RESET -> écrire 1 au bit 4+16 remet PA4 à 0 */
        GPIOA->BSRR = (1U << (4U + 16U));   // BR4 : nCS actif (0)
    } else {
        /* BSRR[15:0]  = bits de SET   -> écrire 1 au bit 4 met PA4 à 1       */
        GPIOA->BSRR = (1U << 4U);           // BS4 : nCS inactif (1)
    }
}

/* Prototypes internes */
static void   config_usart2(void);
static void   init_ADXL(void);
static void   lire_multiple_regADXL(adresse_type reg, int n, uint8_t *dst);

/* -------------------------------------------------------------------------- */
/* Variables GLOBALes liées aux protocoles et états                           */
/* -------------------------------------------------------------------------- */
uint8_t MaeRcp = 0;          // Etat de la machine de réception série (UART RX)
uint8_t entete = 0;          // Dernier octet d'entête reçu (bit7=1 dans le protocole)
uint8_t dataIndex = 0;       // Index de progression dans les données de trame
bool    flagMajCasiers = false; // Demande de mise à jour des casiers (I2C)
bool    flagMajZIF16   = false; // Demande de mise à jour du module ZIF16

/* Consignes 16 bits reçues depuis le PC (via UART) */
uint16_t consigne_leds_vertes = 0;
uint16_t consigne_leds_rouges = 0;
uint16_t consigne_dir        = 0;
uint16_t consigne_etat       = 0;
uint16_t etat_casiers        = 0;

/* ====================== I²C bit-bang sur PB8 (SCL) / PB9 (SDA) =================== */
#define I2C_PORT        GPIOB
#define SCL_PIN         8u
#define SDA_PIN         9u
/* Adresses PCF8574 à ADAPTER selon câblage A2/A1/A0 */
#define PCF_RED_LO_ADDR   0x20  /* LED rouges   bits 0..7  (LSB) */
#define PCF_RED_HI_ADDR   0x21  /* LED rouges   bits 8..15 (MSB) */
#define PCF_GRN_LO_ADDR   0x22  /* LED vertes   bits 0..7  */
#define PCF_GRN_HI_ADDR   0x23  /* LED vertes   bits 8..15 */
#define PCF_SEN_LO_ADDR   0x24  /* CAPTEURS     bits 0..7  */
#define PCF_SEN_HI_ADDR   0x25  /* CAPTEURS     bits 8..15 */

/* -------------------------------------------------------------------------- */
/* FIFO 256 octets pour la TRANSMISSION UART (producteur/consommateur)        */
/* - pw : index d'écriture (producer write)                                   */
/* - pr : index de lecture  (consumer read)                                   */
/* - place_libre : nb de cases disponibles                                    */
/* Le wrap est "naturel" car pw/pr sont des uint8_t (0..255).                 */
/* -------------------------------------------------------------------------- */
uint8_t fifo[256];
uint8_t pw = 0;
uint8_t pr = 0;
int     place_libre = 256;

/* -------------------------------------------------------------------------- */
/* Structure d'accès aux 3 axes (X/Y/Z) de l'ADXL345                          */
/*  - union : accès soit par 3 uint16_t (c.x,c.y,c.z), soit par 6 octets b[]  */
/*  - l'ADXL345 fournit 6 octets consécutifs à partir de DATAX0               */
/* -------------------------------------------------------------------------- */
typedef union access_sensor {
    struct { uint16_t x; uint16_t y; uint16_t z; } c;
    unsigned char b[6];
} struct_xyz_t;

struct_xyz_t capteurs;

/* Masques de bits utilisés par le protocole d'entête série */
#define B7_MASK 0x80   // bit 7
#define B6_MASK 0x40   // bit 6

void init_ZIF16(void)
{
    /* Sécurité : tout en entrée pull-down (PB15..8 et PC7..0) */
    RCC->APB2ENR |= (1<<3) | (1<<4);
    /* PB8..PB15 : CRH, mettre CNF=10 MODE=00 (input PU/PD) + ODR=0 (PD) */
    GPIOB->CRH = 0; /* we'll set per-nibble */
    for (int i=8;i<16;i++){
        uint32_t shift = (i-8)*4;
        GPIOB->CRH &= ~(0xFu<<shift);
        GPIOB->CRH |=  (0x8u<<shift); /* 1000b */
    }
    GPIOB->ODR &= ~(0xFF00u);
    /* PC0..PC7 : CRL, idem */
    for (int i=0;i<8;i++){
        uint32_t shift = i*4;
        GPIOC->CRL &= ~(0xFu<<shift);
        GPIOC->CRL |=  (0x8u<<shift);
    }
    GPIOC->ODR &= ~(0x00FFu);
}
}

uint16_t lire_etat_ZIF16(void)
{
    uint16_t pb = (uint16_t)((GPIOB->IDR >> 8) & 0xFFu);
    uint16_t pc = (uint16_t)(GPIOC->IDR & 0xFFu);
    return (uint16_t)((pb<<8) | pc);
}

void init_I2C_BITBANGING(void)
{
    /* Horloge GPIOB + AFIO */
    RCC->APB2ENR |= (1<<3) | (1<<0);
    /* PB8/PB9 en Open-Drain 2MHz (MODE=10, CNF=01 -> 0b0110 = 0x6) */
    /* PB8 (CRH bits [3:0]) ; PB9 (CRH bits [7:4]) */
    GPIOB->CRH &= ~((0xFu<<0) | (0xFu<<4));
    GPIOB->CRH |=  ((0x6u<<0) | (0x6u<<4));
    /* Relâcher les lignes (niveau haut via pull-ups externes) */
    GPIOB->BSRR = (1u<<SCL_PIN) | (1u<<SDA_PIN);
}

/* -------------------------------------------------------------------------- */
/* USART2 : configuration bas-niveau                                          */
/* - PCLK1 supposé à 36 MHz (si SystemInit met le MCU à 72 MHz, APB1=36 MHz)  */
/* - Baudrate 9600, format 8N1                                                */
/* - PA2=TX AF-PP 2 MHz, PA3=RX input floating                                */
/* -------------------------------------------------------------------------- */
#define USART2_PCLK1 36000000UL
#define USART2_BAUD  9600U

static void config_usart2(void)
{
    uint32_t div, mantissa, fraction;

    /* 1) Activer les horloges GPIOA + AFIO (APB2) */
    RCC->APB2ENR |= (1<<2) | (1<<0);   // IOPAEN=bit2, AFIOEN=bit0

    /* 2) Activer l'horloge USART2 (APB1) */
    
    RCC->APB1ENR |= (1<<17);           // USART2EN=bit17

    /* 3) Configurer PA2/PA3 */
    /* CRL : 4 bits par pin [CNF1:0 | MODE1:0]
       - PA2 (bits 11..8)  = AF Push-Pull, 2 MHz -> nibble 0xA (CNF=10, MODE=10)
       - PA3 (bits 15..12) = Input floating     -> nibble 0x4 (CNF=01, MODE=00) */
    GPIOA->CRL &= ~((0xF<<8) | (0xF<<12));
    GPIOA->CRL |=  (0xA<<8) | (0x4<<12);

    /* Pas de remap USART2 : conserve PA2/PA3 */
    AFIO->MAPR &= ~(1<<3); // clear USART2_REMAP

    /* 4) Configuration USART2 : 9600 8N1, TE+RE */
    USART2->CR1 = 0; USART2->CR2 = 0; USART2->CR3 = 0;

    /* BRR : mantissa<<4 | fraction (oversampling x16) */
    div      = (USART2_PCLK1 + (USART2_BAUD/2U)) / USART2_BAUD; // arrondi
    mantissa = div / 16U;
    fraction = div - (mantissa * 16U);
    USART2->BRR = (mantissa << 4) | (fraction & 0xF);
    /* Si votre APB1 est à 8 MHz (HSI par défaut), utiliser 0x0341 pour 9600bauds */

    /* 1 stop (STOP=00), 8 bits (M=0), pas de parité (PCE=0), TX/RX actifs */
    USART2->CR2 &= ~0x3000;           // STOP[13:12]=00 -> 1 stop
    USART2->CR1 &= ~(0x1000 | 0x0400);// M=0 (8 bits), PCE=0
    USART2->CR1 |=  0x0008 | 0x0004;  // TE | RE

    /* 5) Enable USART */
    USART2->CR1 |= 0x2000;            // UE=1

    /* Lecture SR puis DR pour nettoyer d'éventuels flags pendants */
    (void)USART2->SR;
    (void)USART2->DR;
}


/* -------------------------------------------------------------------------- */
/* SPI1 <-> ADXL345                                                            */
/* Broches :
   - PA5 = SCK (AF Push-Pull)
   - PA6 = MISO (Input floating)
   - PA7 = MOSI (AF Push-Pull)
   - PA4 = nCS  (GPIO Push-Pull, géré logiciel via CS_ADXL)
   Mode SPI : CPOL=1, CPHA=1 (mode 3), MSB first, prescaler /64 (~1.125MHz si PCLK2=72MHz)
   Remarque : ADXL345 supporte jusqu'à 5 MHz, donc /64 est "safe".
/* -------------------------------------------------------------------------- */

/* Echange 8 bits full-duplex : écrit DR, attend RXNE, lit DR */
static uint8_t spi1_txrx(uint8_t v)
{
    SPI1->DR = v;                         // lance l'émission (et la réception)
    while (!(SPI1->SR & SPI_SR_RXNE)) {   // attend 1 octet reçu
        /* wait */
    }
    return (uint8_t)SPI1->DR;             // lire vide RXNE
}

void init_SPI1_ADXL(void)
{
    /* 1) Horloges : GPIOA + AFIO + SPI1 (tous sur APB2) */
    RCC->APB2ENR |= (1<<2) | (1<<0) | (1<<12);

    /* 2) GPIO : nettoyer les nibbles PA4..PA7 (bits 19..16, 23..20, 27..24, 31..28) */
    GPIOA->CRL &= ~(0xFFFF0000);

    /* PA5=SCK  AF-PP 50MHz -> nibble 0xB sur [23:20]  => 0x00B0 0000 */
    /* PA7=MOSI AF-PP 50MHz -> nibble 0xB sur [31:28]  => 0xB000 0000 */
    GPIOA->CRL |= (0x00B00000) | (0xB0000000);

    /* PA6=MISO input floating -> nibble 0x4 sur [27:24] */
    GPIOA->CRL |= 0x04000000;

    /* PA4=nCS GPIO Push-Pull 50MHz -> nibble 0x3 sur [19:16] */
    GPIOA->CRL |= 0x00030000;

    /* nCS inactif (haut) au repos */
    GPIOA->BSRR = (1U << 4);

    /* 3) SPI1 : MAITRE, NSS logiciel, mode 3, BR=/64 */
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;

    /* 0x032F = MSTR(0x0004) | SSM(0x0200) | SSI(0x0100) | CPOL(0x0002) | CPHA(0x0001) | BR2(0x0020) | BR0(0x0008) */
    SPI1->CR1 = 0x032F;
    SPI1->CR1 |= 0x0040; // SPE=1 (enable)

    /* purge initiale (au cas où) */
    (void)SPI1->SR; (void)SPI1->DR;
}

/* -------------------------------------------------------------------------- */
/* ADXL345 : primitives registre                                              */
/* - Ecriture simple : R/W=0, MB=0 -> adresse = reg & 0x3F                    */
/* - Lecture  simple : R/W=1, MB=0 -> adresse = 0x80 | (reg & 0x3F)          */
/* - Lecture rafale : R/W=1, MB=1 -> adresse = 0xC0 | (reg & 0x3F)           */
/* -------------------------------------------------------------------------- */

/* Ecrit un registre ADXL345 (adresse puis octet de donnée) */
void config_regADXL(adresse_type ADXL345_REG, val_type ADXL345_VAL_REG)
{
    uint8_t addr = (uint8_t)(ADXL345_REG & 0x3F); // écriture simple

    CS_ADXL(SELECT);                 // nCS bas
    (void)spi1_txrx(addr);           // adresse
    (void)spi1_txrx((uint8_t)ADXL345_VAL_REG); // donnée
    CS_ADXL(RELEASE);                // nCS haut
}

/* Lit un registre ADXL345 (1 octet) */
type_retour lire_regADXL(adresse_type ADXL345_REG)
{
    uint8_t addr = 0x80 | (uint8_t)(ADXL345_REG & 0x3F); // lecture simple
    uint8_t val;

    CS_ADXL(SELECT);
    (void)spi1_txrx(addr);      // envoyer l'adresse
    val = spi1_txrx(0xFF);      // dummy write pour clocker la lecture
    CS_ADXL(RELEASE);

    return (type_retour)val;
}

/* Lit n octets consécutifs à partir du registre "reg" (rafale si n>1) */
static void lire_multiple_regADXL(adresse_type reg, int n, uint8_t *dst)
{
    uint8_t addr = 0x80 | (uint8_t)(reg & 0x3F);
    int i;

    if (n > 1) {
        addr |= 0x40; // MB=1 -> mode rafale
    }

    CS_ADXL(SELECT);
    (void)spi1_txrx(addr);
    for (i = 0; i < n; i++) {
        dst[i] = spi1_txrx(0xFF); // dummy -> lit un octet
    }
    CS_ADXL(RELEASE);
}

/* -------------------------------------------------------------------------- */
/* ADXL345 : configuration complète + INT                                     */
/* - Configure PB0 en entrée pull-up pour lire INT1 (*adapter si besoin*)     */
/* - Initialise SPI1, vérifie DEVID, programme les registres essentiels       */
/* - DATA_FORMAT : 4-fils, full-res, ±16g, INT actives bas                    */
/* - POWER_CTL : wake puis measure                                            */
/* - Interrupts : DATA_READY sur INT1                                         */
/* -------------------------------------------------------------------------- */

/* Choix de la broche d'interruption ADXL345 : INT1 sur PB0 */
#define ADXL_INT_GPIO  GPIOB
#define ADXL_INT_PIN   0
#define INT_ADXL (!(ADXL_INT_GPIO->IDR & (1U << ADXL_INT_PIN))) // actif bas

void init_ADXL(void)
{
    uint8_t devid;

    /* 1) GPIOB clock + PB0 en input pull-up (MODE=00, CNF=10) */
    RCC->APB2ENR |= (1<<3);                 // IOPBEN
    ADXL_INT_GPIO->CRL &= ~(0xF << 0);      // clear nibble PB0
    ADXL_INT_GPIO->CRL |=  (0x8 << 0);      // CNF=10, MODE=00
    ADXL_INT_GPIO->ODR |=  (1U << ADXL_INT_PIN); // pull-up

    /* 2) SPI1 + broches SPI */
    init_SPI1_ADXL();

    /* 3) Lire DEVID (doit valoir 0xE5) */
    devid = (uint8_t)lire_regADXL(ADXL345_DEVID);
    (void)devid; // placer un breakpoint pour vérifier

    /* 4) Configuration des registres ADXL345 */
    /* Sortie veille puis passage en mesure */
    config_regADXL(ADXL345_POWER_CTL, 0x00);
    config_regADXL(ADXL345_POWER_CTL, 0x08); // Measure=1

    /* DATA_FORMAT : SPI 4-fils, full-res, right-justified, ±16g, INT actives bas
       0b0010_1011 = 0x2B */
    config_regADXL(ADXL345_DATA_FORMAT, 0x2B);

    /* Activité / inactivité (exemple) */
    config_regADXL(ADXL345_ACT_INACT_CTL, 0xFF);
    config_regADXL(ADXL345_THRESH_ACT,     16); // ~1 g (à adapter)
    config_regADXL(ADXL345_THRESH_INACT,    8); // ~0.5 g
    config_regADXL(ADXL345_TIME_INACT,      1); // 1 s

    /* Détection de chocs (tap) */
    config_regADXL(ADXL345_TAP_AXES,   0x0F);  // X/Y/Z
    config_regADXL(ADXL345_THRESH_TAP, 0xA0);  // ~10 g
    config_regADXL(ADXL345_DUR,        16);    // ~10 ms
    config_regADXL(ADXL345_LATENT,     0x00);
    config_regADXL(ADXL345_WINDOW,     0x00);  // si présent

    /* Free-fall */
    config_regADXL(ADXL345_THRESH_FF,  0x09);  // ~0.6 g
    config_regADXL(ADXL345_TIME_FF,    10);    // 50 ms

    /* Débit & FIFO : 100 Hz, STREAM, watermark 16 */
    config_regADXL(ADXL345_BW_RATE,    0x0A);  // 100 Hz
    config_regADXL(ADXL345_FIFO_CTL,   0x90);  // STREAM | WM=16

    /* Interrupts : DATA_READY -> INT1 (bit=0), autres sur INT2 (bit=1) */
    config_regADXL(ADXL345_INT_MAP,    0x7F);  // DATA_READY sur INT1
    config_regADXL(ADXL345_INT_ENABLE, 0xDF);  // enable DRDY/ACT/INACT/FF/TAP...

    /* 5) Nettoyage : lecture INT_SOURCE */
    (void)lire_regADXL(ADXL345_INT_SOURCE);
}


/* -------------------------------------------------------------------------- */
/* Réception UART : gere_serial2()                                            */
/* - Lit SR puis DR (dans cet ordre) -> efface RXNE                           */
/* - Détecte erreurs ligne (PE/FE/NE/ORE) et réinitialise l'état              */
/* - Si bit7 du byte reçu = 1 -> c'est une ENTETE ; sinon ce sont des DONNEES */
/* - Les 4 bits bas de l'entête reportent les bits7 des 4 octets de données   */
/* - Remplit consigne LEDs/ZIF16 et lève flagMaj... quand une trame complète  */
/* -------------------------------------------------------------------------- */
void gere_serial2(void)
{
    uint8_t status = USART2->SR;     // lire SR d'abord
    uint8_t ch_recu = USART2->DR;    // puis DR -> efface RXNE

    /* Détection d'erreurs ligne : PE/FE/NE/ORE */
    if (status & (0x0008 | 0x0004 | 0x0002 | 0x0001)) {
        MaeRcp = 0;
        return;
    }

    if (ch_recu & B7_MASK) {
        /* === ENTETE === (bit7=1) */
        entete = ch_recu;
        dataIndex = 0;
        if (ch_recu & B6_MASK) {
            MaeRcp = 1; // trame "casiers" (LEDs)
        } else {
            MaeRcp = 5; // trame "ZIF16"
        }
    } else {
        /* === DONNEES === (bit7=0) */
        switch (MaeRcp) {
            /* Etat 0 : attente d'entête -> on ignore */
            case 0:
                break;

            /* ----- CASIERS : deux mots (verts, rouges) ----- */
            case 1: { // MSB leds vertes
                if (entete & (1 << 0)) ch_recu |= B7_MASK; // réinjecter bit7 reporté
                consigne_leds_vertes = (consigne_leds_vertes & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 2;
                break;
            }
            case 2: { // LSB leds vertes
                if (entete & (1 << 1)) ch_recu |= B7_MASK;
                consigne_leds_vertes = (consigne_leds_vertes & 0xFF00) | ch_recu;
                MaeRcp = 3;
                break;
            }
            case 3: { // MSB leds rouges
                if (entete & (1 << 2)) ch_recu |= B7_MASK;
                consigne_leds_rouges = (consigne_leds_rouges & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 4;
                break;
            }
            case 4: { // LSB leds rouges -> trame casiers complète
                if (entete & (1 << 3)) ch_recu |= B7_MASK;
                consigne_leds_rouges = (consigne_leds_rouges & 0xFF00) | ch_recu;
                flagMajCasiers = true;
                MaeRcp = 0;
                break;
            }

            /* ----- ZIF16 : deux mots (dir, etat) ----- */
            case 5: { // MSB dir
                if (entete & (1 << 0)) ch_recu |= B7_MASK;
                consigne_dir = (consigne_dir & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 6;
                break;
            }
            case 6: { // LSB dir
                if (entete & (1 << 1)) ch_recu |= B7_MASK;
                consigne_dir = (consigne_dir & 0xFF00) | ch_recu;
                MaeRcp = 7;
                break;
            }
            case 7: { // MSB etat
                if (entete & (1 << 2)) ch_recu |= B7_MASK;
                consigne_etat = (consigne_etat & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 8;
                break;
            }
            case 8: { // LSB etat -> trame ZIF16 complète
                if (entete & (1 << 3)) ch_recu |= B7_MASK;
                consigne_etat = (consigne_etat & 0xFF00) | ch_recu;
                flagMajZIF16 = true;
                MaeRcp = 0;
                break;
            }

            default:
                MaeRcp = 0;
                break;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* I2C bit-banging : squelettes                                                */
/* A compléter : générer Start/Stop, écrire/lire bits, gérer ACK/NACK         */
/* -------------------------------------------------------------------------- */
void I2C_Delay(void)
{
    volatile int i;
    for (i = 0; i < 10; i++) {
        /* petit délai */
    }
}

void I2C_Start(void)
{
    /* SDA:1, SCL:1 -> START: SDA 1->0 pendant SCL=1 */
    GPIOB->BSRR = (1u<<SDA_PIN) | (1u<<SCL_PIN);
    I2C_Delay();
    GPIOB->BSRR = (1u<<(SDA_PIN+16u));  /* SDA low */
    I2C_Delay();
    GPIOB->BSRR = (1u<<(SCL_PIN+16u));  /* SCL low */
    I2C_Delay();
}
void I2C_Stop(void)
{
    /* STOP: SDA 0->1 pendant SCL=1 */
    GPIOB->BSRR = (1u<<(SDA_PIN+16u));  /* SDA low */
    I2C_Delay();
    GPIOB->BSRR = (1u<<SCL_PIN);        /* SCL high */
    I2C_Delay();
    GPIOB->BSRR = (1u<<SDA_PIN);        /* SDA high (release) */
    I2C_Delay();
}
void I2C_WriteBit(uint8_t bit)
{
    if (bit) GPIOB->BSRR = (1u<<SDA_PIN);
    else     GPIOB->BSRR = (1u<<(SDA_PIN+16u));
    I2C_Delay();
    GPIOB->BSRR = (1u<<SCL_PIN);
    I2C_Delay();
    GPIOB->BSRR = (1u<<(SCL_PIN+16u));
    I2C_Delay();
}
uint8_t I2C_ReadBit(void)
{
    uint8_t bit;
    /* Relâcher SDA (entrée via OD) */
    GPIOB->BSRR = (1u<<SDA_PIN);
    I2C_Delay();
    GPIOB->BSRR = (1u<<SCL_PIN);
    I2C_Delay();
    bit = (uint8_t)((GPIOB->IDR >> SDA_PIN) & 1u);
    GPIOB->BSRR = (1u<<(SCL_PIN+16u));
    I2C_Delay();
    return bit;
}
uint8_t I2C_WriteByte(uint8_t data)
{
    for (int i = 7; i >= 0; --i) {
        I2C_WriteBit((uint8_t)((data>>i) & 1u));
    }
    /* Lire ACK (0=ACK) -> renvoyer 1 si ACK reçu */
    return (I2C_ReadBit() == 0u) ? 1u : 0u;
}
uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t v = 0;
    for (int i = 7; i >= 0; --i) {
        v <<= 1;
        v |= I2C_ReadBit();
    }
    /* ack=1 -> envoyer ACK(0) ; ack=0 -> envoyer NACK(1) */
    I2C_WriteBit(ack ? 0u : 1u);
    return v;
}

void I2C_write_PCF8574(uint8_t adresse, uint8_t data)
{
    I2C_Start();
    (void)I2C_WriteByte( (uint8_t)((adresse<<1) | 0u) );
    (void)I2C_WriteByte(data);
    I2C_Stop();
}
uint8_t I2C_read_PCF8574(uint8_t adresse)
{
    uint8_t d;
    I2C_Start();
    (void)I2C_WriteByte( (uint8_t)((adresse<<1) | 1u) );
    d = I2C_ReadByte(0u); /* NACK fin */
    I2C_Stop();
    return d;
}

void maj_leds_casiers(void)
{
    /* PCF8574 actif bas -> on écrit l'inversion des consignes */
    uint8_t v_lo = (uint8_t)(~consigne_leds_vertes & 0xFFu);
    uint8_t v_hi = (uint8_t)((~consigne_leds_vertes >> 8) & 0xFFu);
    uint8_t r_lo = (uint8_t)(~consigne_leds_rouges & 0xFFu);
    uint8_t r_hi = (uint8_t)((~consigne_leds_rouges >> 8) & 0xFFu);

    I2C_write_PCF8574(PCF_GRN_LO_ADDR, v_lo);
    I2C_write_PCF8574(PCF_GRN_HI_ADDR, v_hi);
    I2C_write_PCF8574(PCF_RED_LO_ADDR, r_lo);
    I2C_write_PCF8574(PCF_RED_HI_ADDR, r_hi);
}
void lire_etat_casiers(void)
{
    /* Lire 2 PCF "capteurs" puis inverser : 1 = contact fermé */
    uint8_t lo = I2C_read_PCF8574(PCF_SEN_LO_ADDR);
    uint8_t hi = I2C_read_PCF8574(PCF_SEN_HI_ADDR);
    etat_casiers = (uint16_t)(((uint16_t)hi<<8) | lo);
    etat_casiers = (uint16_t)(~etat_casiers);
}
void envoyer_trame_casiers(void)
{
    /* Réponse 0xC0 + 2 octets (b7=0), b7 des data dans entête b0..b1 */
    uint8_t p1 = (uint8_t)(etat_casiers >> 8);
    uint8_t p2 = (uint8_t)(etat_casiers & 0xFFu);
    uint8_t ent = (uint8_t)(0xC0 | ((p1>>7)&1u) | (((p2>>7)&1u)<<1));
    if (fifo_peut_ecrire(3)) {
        fifo_put(ent);
        fifo_put((uint8_t)(p1 & 0x7Fu));
        fifo_put((uint8_t)(p2 & 0x7Fu));
    }
}
void envoyer_trame_ZIF16(void)
{
    uint16_t w = lire_etat_ZIF16();
    uint8_t p1 = (uint8_t)(w >> 8);
    uint8_t p2 = (uint8_t)(w & 0xFFu);
    uint8_t ent = (uint8_t)(0x80 | ((p1>>7)&1u) | (((p2>>7)&1u)<<1));
    if (fifo_peut_ecrire(3)) {
        fifo_put(ent);
        fifo_put((uint8_t)(p1 & 0x7Fu));
        fifo_put((uint8_t)(p2 & 0x7Fu));
    }
}

/* -------------------------------------------------------------------------- */
/* Initialisation globale de la carte                                         */
/* -------------------------------------------------------------------------- */
void init_proc(void)
{
    init_ZIF16();          // placeholder
    init_I2C_BITBANGING(); // à implémenter
    config_usart2();        // USART2 (PA2/PA3) 9600 8N1
    init_SPI1_ADXL();      // SPI1 + GPIO
    init_ADXL();       // configuration ADXL345 (écritures registres)
}

/* -------------------------------------------------------------------------- */
/* FIFO : helpers producteur                                                  */
/* -------------------------------------------------------------------------- */
static int fifo_peut_ecrire(int n) { return place_libre >= n; }

static void fifo_put(uint8_t b) {
    fifo[pw] = b;
    pw = (uint8_t)(pw + 1);   // wrap 0..255
    place_libre--;
}

static int fifo_put_block(const uint8_t *blk, int n) {
    int i;
    if (!fifo_peut_ecrire(n)) return 0;
    for (i = 0; i < n; i++) fifo_put(blk[i]);
    return 1;
}

/* Lecture des 6 octets capteurs (X/Y/Z) */
void read_ADXL_sensors(uint8_t *dst6) {
    lire_multiple_regADXL(ADXL345_DATAX0, 6, dst6);
}

/* Compression "13 bits signés" : renvoie deux octets b7=0 chacun
   - hi6 = bits [12:7] (6 bits)
   - lo7 = bits [6:0]  (7 bits) */
static void pack13(int16_t v, uint8_t *hi6, uint8_t *lo7) {
    uint16_t s13 = (uint16_t)v & 0x1FFF;   // garder 13 bits (deux-complements)
    *hi6 = (uint8_t)((s13 >> 7) & 0x3F);
    *lo7 = (uint8_t)( s13       & 0x7F);
}

/* Construit la trame 7 octets : 0xF0 + X(2) + Y(2) + Z(2) compressés */
void envoi_trame_accelero(void)
{
    /* Recomposer les 3 int16_t à partir des 6 octets lus (LSB->MSB) */
    int16_t x = (int16_t)((int16_t)((int8_t)capteurs.b[1])<<8 | capteurs.b[0]);
    int16_t y = (int16_t)((int16_t)((int8_t)capteurs.b[3])<<8 | capteurs.b[2]);
    int16_t z = (int16_t)((int16_t)((int8_t)capteurs.b[5])<<8 | capteurs.b[4]);

    uint8_t trame[7];
    trame[0] = 0xF0;             // entête non conflictuelle (pas 0x8n / 0xCn)

    pack13(x, &trame[1], &trame[2]);
    pack13(y, &trame[3], &trame[4]);
    pack13(z, &trame[5], &trame[6]);

    (void)fifo_put_block(trame, 7); // pousse si assez de place
}

/* Petit outil de test : remplir la FIFO avec n×0x55 (pattern visible à 9600) */
static void fifo_push_0x55_burst(uint16_t n)
{
    while (n--) {
        if (!fifo_peut_ecrire(1)) break;
        fifo_put(0x55);
    }
}



int main(void)
{
    uint16_t etat_ZIF16 = 0;

    init_proc();

    /* Option DEBUG : pré-remplir la FIFO avec 0x55 (mesure au terminal/oscillo) */
    fifo_push_0x55_burst(256);

    while (1) {

        /* --- Trajets "casiers" : déclenchés après réception d'une trame UART --- */
        if (flagMajCasiers) {
            flagMajCasiers = false;
            /* A faire : écrire/relire via I2C PCF8574 et construire la trame */
            maj_leds_casiers();
            lire_etat_casiers();
            envoyer_trame_casiers();
        }

        /* --- Trajets "ZIF16" : idem casiers --- */
        if (flagMajZIF16) {
            flagMajZIF16 = false;
            init_ZIF16();                          // mise à jour des pattes
            etat_ZIF16 = lire_etat_ZIF16();        // lecture de l'état
            (void)etat_ZIF16;
            envoyer_trame_ZIF16();                 // envoi vers PC via UART
        }

        /* --- Réception série disponible ? -> parser la trame reçue --- */
        if (USART2->SR & USART_SR_RXNE) {
            gere_serial2();
        }

        /* --- Emission série : DEPILER FIFO si TXE=1 --- */
#ifdef UART_TEST
        /* Mode test : envoyer 0x55 en continu sur TXE */
        if (USART2->SR & USART_SR_TXE) {
            USART2->DR = 0x55;
        }
#else
        if (USART2->SR & USART_SR_TXE) {
            if (place_libre < (int)sizeof(fifo)) {
                USART2->DR = fifo[pr];
                pr = (uint8_t)(pr + 1);  // wrap naturel
                place_libre++;
            }
        }
#endif

        /* --- Si nouvelle donnée ADXL prête (IRQ active bas), lire et pousser --- */
        if (INT_ADXL) {
            read_ADXL_sensors(&capteurs.b[0]);
            envoi_trame_accelero();
        }

        /* --- Option DEBUG : réalimenter périodiquement la FIFO en 0x55 --- */
        if (place_libre >= 32) {
            fifo_push_0x55_burst(32);
        }
    }
}
