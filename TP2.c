#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>


#include "ADXL345.h"

// === Variables globales ===
uint8_t MaeRcp = 0;
uint8_t entete = 0;
uint8_t dataIndex = 0;
bool flagMajCasiers = false;
bool flagMajZIF16 = false;

// === Consignes 16 bits ===
uint16_t consigne_leds_vertes = 0;
uint16_t consigne_leds_rouges = 0;
uint16_t consigne_dir = 0;
uint16_t consigne_etat = 0;
uint16_t etat_casiers =0;


uint8_t fifo[256];
uint8_t pw =0;
uint8_t pr =0;
int place_libre = 256;

typedef union access_sensor {//16 bit

        struct {
            uint16_t x; 
            uint16_t y; 
					  uint16_t z; 
        } c;
        unsigned char b[6];
    } struct_xyz_t;

struct_xyz_t capteurs;


#define B7_MASK 0x80
#define B6_MASK 0x40


void init_ZIF16(void)
{uint8_t boucle;
 uint32_t choix_INOUT;
 uint8_t	choix_etat;	
for(boucle = 0; boucle<8; boucle++)
	{ choix_INOUT = 	;
		choix_etat = 		 ;
	  
	}
for(boucle = 8; boucle<16; boucle++)
	{ choix_INOUT =  ;
	  choix_etat =   ;
	 
	}
}

uint16_t lire_etat_ZIF16(void)
{
	return ;
}

void  init_I2C_BITBANGING(void)
{ 

}	
#define USART2_PCLK1     36000000UL
#define USART2_BAUD      9600U
static void config_usart2(void)
{
   /* 1) Clocks GPIOA + AFIO (pour config AF des broches) */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

    /* 2) Clock périphérique USART2 (bus APB1) */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* 3) Configuration des broches
         - PA2 = USART2_TX : Alternate Function Push-Pull, 2 MHz
         - PA3 = USART2_RX : Entrée flottante
       (CRL: Pin0..7 ; champ de 4 bits par pin : [CNF1:0 | MODE1:0]) */
    GPIOA->CRL &= ~((GPIO_CRL_MODE2 | GPIO_CRL_CNF2) |
                    (GPIO_CRL_MODE3 | GPIO_CRL_CNF3));
    // PA2: MODE2=10 (2 MHz), CNF2=10 (AF push-pull)
    GPIOA->CRL |=  (GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1);
    // PA3: MODE3=00 (input), CNF3=01 (entrée flottante)
    GPIOA->CRL |=  (GPIO_CRL_CNF3_0);

    // Pas de remap : PA2/PA3 par défaut
    AFIO->MAPR &= ~AFIO_MAPR_USART2_REMAP;

    /* 4) Configuration USART2 : 9600 8N1, TX+RX */
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    // Baudrate (oversampling x16) : BRR = mantissa<<4 | fraction
    uint32_t div = (USART2_PCLK1 + (USART2_BAUD/2U)) / USART2_BAUD; // arrondi
    uint32_t mantissa = div / 16U;
    uint32_t fraction = div - (mantissa * 16U);
    USART2->BRR = (mantissa << 4) | (fraction & 0xF);
    // Équivalent fixe si PCLK1=36 MHz : USART2->BRR = 0x0EA6;

    // 8 bits, sans parité, 1 stop (STOP=00), TX et RX actifs
    USART2->CR2 &= ~USART_CR2_STOP;
    USART2->CR1 |=  USART_CR1_TE | USART_CR1_RE;

    // 5) Enable USART
    USART2->CR1 |= USART_CR1_UE;

    // Lecture SR puis DR pour nettoyer d’éventuels flags pendants
    (void)USART2->SR;
    (void)USART2->DR;

  
}
// ===== SPI1 <-> ADXL345 =====
// Broches (Nucleo F103RB):
// PA5 = SCK (AF Push-Pull)   | PA6 = MISO (entrée flottante)
// PA7 = MOSI (AF Push-Pull)  | PA4 = nCS (GPIO sortie P-P, pilotée logiciel)

static inline uint8_t spi1_txrx(uint8_t v)
{
    SPI1->DR = v;                                // émettre
    while (!(SPI1->SR & SPI_SR_RXNE)) { /* wait */ } // attendre un octet reçu
    return (uint8_t)SPI1->DR;                    // lire le tampon (vide le flag)
}

void init_SPI1_ADXL(void)
{
     // 1) Horloges: GPIOA + AFIO + SPI1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN;

    // 2) GPIO:
    // PA5 (SCK)  = AF Push-Pull, 50 MHz
    // PA7 (MOSI) = AF Push-Pull, 50 MHz
    // PA6 (MISO) = Entrée flottante
    // PA4 (nCS)  = Sortie push-pull, 50 MHz (pilotage logiciel)
    GPIOA->CRL &=
        ~((GPIO_CRL_MODE5 | GPIO_CRL_CNF5) |
          (GPIO_CRL_MODE6 | GPIO_CRL_CNF6) |
          (GPIO_CRL_MODE7 | GPIO_CRL_CNF7) |
          (GPIO_CRL_MODE4 | GPIO_CRL_CNF4));

    // MODE=11 (50MHz), CNF=10 (AF P-P) pour PA5 et PA7
    GPIOA->CRL |= (GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1) |
                  (GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1);
    // PA6: MODE=00 (input), CNF=01 (entrée flottante)
    GPIOA->CRL |= GPIO_CRL_CNF6_0;
    // PA4: MODE=11 (50MHz), CNF=00 (GPIO P-P)
    GPIOA->CRL |= (GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1);
    // nCS inactif (haut)
    GPIOA->BSRR = (1U << 4);

    // 3) SPI1 configuration:
    // - Master, 8 bits, MSB first
    // - CPOL=1, CPHA=1  (ADXL345 = SPI mode 3)
    // - NSS logiciel (SSM=1, SSI=1) car nCS géré par PA4
    // - Vitesse: PCLK2/64 (~72MHz/64 ≈ 1.125 MHz) < 5 MHz max ADXL345
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;

    SPI1->CR1 =
        SPI_CR1_MSTR       |        // maître
        SPI_CR1_SSM        |        // NSS logiciel
        SPI_CR1_SSI        |        // NSS haut interne (évite MODF)
        SPI_CR1_CPOL       |        // CPOL=1
        SPI_CR1_CPHA       |        // CPHA=1 (mode 3)
        SPI_CR1_BR_2 | SPI_CR1_BR_0; // /64 (BR=101)

    // Activer SPI
    SPI1->CR1 |= SPI_CR1_SPE;

    // purge initiale (au cas où)
    (void)SPI1->SR; (void)SPI1->DR;
 
}

// Écrit un registre ADXL345 : adresse (R/W=0, MB=0) puis la valeur
void config_regADXL(adresse_type ADXL345_REG, val_type ADXL345_VAL_REG)
{
    // ADXL345 : bit7 = R/W, bit6 = MB (multi-byte)
    // Pour une écriture simple : R/W=0, MB=0 → on garde seulement reg[5:0]
    uint8_t addr = (uint8_t)(ADXL345_REG & 0x3F);

    // Sélectionner le capteur
    CS_ADXL(SELECT);

    // Envoyer l’adresse puis la donnée (lecture de retour ignorée)
    (void)spi1_txrx(addr);
    (void)spi1_txrx((uint8_t)ADXL345_VAL_REG);

    // Libérer le capteur
    CS_ADXL(RELEASE);
}
// Lecture d’un registre ADXL345 via SPI1 (mode 3)
// R/W=1 (bit7), MB=0 (bit6) → lecture simple, 1 octet
type_retour lire_regADXL(adresse_type ADXL345_REG)
{
    uint8_t addr = 0x80 | (uint8_t)(ADXL345_REG & 0x3F);  // 1xxx xxxx

    CS_ADXL(SELECT);                 // nCS bas
    (void)spi1_txrx(addr);           // envoi adresse (retour ignoré)
    uint8_t val = spi1_txrx(0xFF);   // dummy write → lit la donnée
    CS_ADXL(RELEASE);                // nCS haut

    return (type_retour)val;
}

     
     
     
// Choix de la broche d'interruption ADXL345 (à adapter si besoin)
// Ici : INT1 sur PB0  (entrée pull-up)
#define ADXL_INT_GPIO      GPIOB
#define ADXL_INT_PIN       0

void init_ADXL(void)
{
    /* --- 1) GPIO pour broche d'interruption ADXL : PB0 input pull-up --- */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                 // horloge GPIOB
    // PB0 : MODE=00 (input), CNF=10 (entrée pull-up/pull-down)
    ADXL_INT_GPIO->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    ADXL_INT_GPIO->CRL |=  (GPIO_CRL_CNF0_1);
    ADXL_INT_GPIO->ODR |=  (1U << ADXL_INT_PIN);        // pull-up

    /* --- 2) SPI1 + broches SPI --- */
    init_SPI1_ADXL();                                   // déjà fourni

    /* --- 3) Vérifier l’ID du capteur (optionnel mais utile) --- */
    uint8_t devid = (uint8_t)lire_regADXL(ADXL345_DEVID);
    // L’ID attendu de l’ADXL345 = 0xE5. Tu peux poser un breakpoint ici si besoin.

    /* --- 4) Configuration des registres ADXL345 via l’outil 1 --- */
    // Sortie du mode veille puis passage en mesure
    config_regADXL(ADXL345_POWER_CTL, 0x00);            // Wake-up
    config_regADXL(ADXL345_POWER_CTL, 0x08);            // Measure=1

    // Format des données : SPI 4-fils, INT actives à l’état bas (/INT),
    // Full-Resolution, justification droite, ±16g
    // 0b0010_1011 = 0x2B
    config_regADXL(ADXL345_DATA_FORMAT, 0x2B);

    // Activité / inactivité (exemple TP) : AC sur XYZ, seuils et temps
    config_regADXL(ADXL345_ACT_INACT_CTL, 0xFF);
    config_regADXL(ADXL345_THRESH_ACT,     16);         // ~1 g
    config_regADXL(ADXL345_THRESH_INACT,    8);         // ~0.5 g
    config_regADXL(ADXL345_TIME_INACT,      1);         // 1 s

    // Détection de chocs (tap)
    config_regADXL(ADXL345_TAP_AXES,   0x0F);           // X/Y/Z
    config_regADXL(ADXL345_THRESH_TAP, 0xA0);           // ~10 g
    config_regADXL(ADXL345_DUR,        16);             // ~10 ms
    config_regADXL(ADXL345_LATENT,     0x00);
    config_regADXL(ADXL345_WINDOW,     0x00);           // (si présent)

    // Free-fall
    config_regADXL(ADXL345_THRESH_FF,  0x09);           // ~0.6 g
    config_regADXL(ADXL345_TIME_FF,    10);             // 50 ms

    // Débit & FIFO : 100 Hz, mode STREAM, watermark à 16, trig sur INT1
    config_regADXL(ADXL345_BW_RATE,    0x0A);           // 100 Hz
    config_regADXL(ADXL345_FIFO_CTL,   0x90);           // STREAM | WM=16

    // Interrupts : mapper DATA_READY sur INT1 (0), le reste sur INT2 (1)
    // INT_MAP: bit=0 -> INT1, bit=1 -> INT2
    config_regADXL(ADXL345_INT_MAP,    0x7F);           // DATA_READY -> INT1
    // Activer DATA_READY, ACTIVITY, INACTIVITY, FREE_FALL, TAP, etc.
    config_regADXL(ADXL345_INT_ENABLE, 0xDF);           // 1101_1111

    /* --- 5) (Option) lecture du INT_SOURCE pour nettoyer --- */
    (void)lire_regADXL(ADXL345_INT_SOURCE);
}





void gere_serial2(void) {
	uint8_t status = USART2->SR;
	uint8_t ch_recu = USART2->DR;
	if(status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {MaeRcp=0;return;}
    if (ch_recu & B7_MASK) {
        // === ENT�TE ===
        entete = ch_recu;
        dataIndex = 0;

        if (ch_recu & B6_MASK) {
            // Message pour les LEDs (casiers)
            MaeRcp = 1;
        } else {
            // Message pour ZIF16
            MaeRcp = 5;
        }
    } else {
        // === DONN�ES ===
        switch (MaeRcp) {
            case 0:
                // Attente d'ent�te
                break;

            // === CASIERS : LEDs vertes/rouges ===
            case 1: {
                if (entete & (1 << 0)) ch_recu |= B7_MASK;
                consigne_leds_vertes = (consigne_leds_vertes & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 2;
                break;
            }
            case 2: {
                if (entete & (1 << 1)) ch_recu |= B7_MASK;
                consigne_leds_vertes = (consigne_leds_vertes & 0xFF00) | ch_recu;
                MaeRcp = 3;
                break;
            }
            case 3: {
                if (entete & (1 << 2)) ch_recu |= B7_MASK;
                consigne_leds_rouges = (consigne_leds_rouges & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 4;
                break;
            }
            case 4: {
                if (entete & (1 << 3)) ch_recu |= B7_MASK;
                consigne_leds_rouges = (consigne_leds_rouges & 0xFF00) | ch_recu;
                flagMajCasiers = true;
                MaeRcp = 0;
                break;
            }

            // === ZIF16 : direction et �tat ===
            case 5: {
                if (entete & (1 << 0)) ch_recu |= B7_MASK;
                consigne_dir = (consigne_dir & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 6;
                break;
            }
            case 6: {
                if (entete & (1 << 1)) ch_recu |= B7_MASK;
                consigne_dir = (consigne_dir & 0xFF00) | ch_recu;
                MaeRcp = 7;
                break;
            }
            case 7: {
                if (entete & (1 << 2)) ch_recu |= B7_MASK;
                consigne_etat = (consigne_etat & 0x00FF) | ((uint16_t)ch_recu << 8);
                MaeRcp = 8;
                break;
            }
            case 8: {
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

void I2C_Delay(void) {volatile int i;
    for ( i = 0; i < 10; i++);
}

// Start condition
void I2C_Start(void) {
 
}

// Stop condition
void I2C_Stop(void) {
 
}

// Send 1 bit
void I2C_WriteBit(uint8_t bit) {
  
}

// Read 1 bit
uint8_t I2C_ReadBit(void) {

}

// Send 1 byte (returns ACK=0 / NACK=1)
uint8_t I2C_WriteByte(uint8_t data) 
{
}

// Read 1 byte and send ACK/NACK
uint8_t I2C_ReadByte(uint8_t ack) 
{
}
void I2C_write_PCF8574(uint8_t adresse, uint8_t data)
{ 
}

uint8_t I2C_read_PCF8574(uint8_t adresse)
{	
}
void maj_leds_casiers(void)
{
	} 
void lire_etat_casiers(void)
{
}


void init_proc(void)
{
 init_ZIF16();
 init_I2C_BITBANGING();
 init_serial2();
 init_SPI1_ADXL();
 init_adxl_345();// fonction donn�e � condition d'�crire 	config_regADXL(unsigned char reg, unsigned char data) 
}
static inline int fifo_peut_ecrire(int n) { return place_libre >= n; }

static inline void fifo_put(uint8_t b) {
    fifo[pw] = b;
    pw = (uint8_t)(pw + 1);   // 256 => auto-wrap
    place_libre--;
}

// pousse un bloc si assez de place
static inline int fifo_put_block(const uint8_t *blk, int n) {
    if (!fifo_peut_ecrire(n)) return 0;
    for (int i = 0; i < n; i++) fifo_put(blk[i]);
    return 1;
}
void read_ADXL_sensors(uint8_t *dst6) {
    lire_multiple_regADXL(ADXL345_DATAX0, 6, dst6);
}
static inline void pack13(int16_t v, uint8_t *hi6, uint8_t *lo7) {
    uint16_t s13 = (uint16_t)v & 0x1FFF;   // deux-complements 13 bits
    *hi6 = (uint8_t)((s13 >> 7) & 0x3F);   // 6 bits, b7=0
    *lo7 = (uint8_t)( s13       & 0x7F);   // 7 bits, b7=0
}

void envoi_trame_accelero(void)
{
    // capteurs.b[0..5] remplis par read_ADXL_sensors()
    int16_t x = (int16_t)((int16_t)((int8_t)capteurs.b[1])<<8 | capteurs.b[0]);
    int16_t y = (int16_t)((int16_t)((int8_t)capteurs.b[3])<<8 | capteurs.b[2]);
    int16_t z = (int16_t)((int16_t)((int8_t)capteurs.b[5])<<8 | capteurs.b[4]);

    uint8_t trame[7];
    trame[0] = 0xF0;

    pack13(x, &trame[1], &trame[2]);
    pack13(y, &trame[3], &trame[4]);
    pack13(z, &trame[5], &trame[6]);

    (void)fifo_put_block(trame, 7);     // si pas assez de place, trame ignorée
}

//============================================================================================test
static inline void fifo_push_0x55_burst(uint16_t n)
{
    while (n--) {
        if (!fifo_peut_ecrire(1)) break;
        fifo_put(0x55);
    }
}
//============================================================================================test

int main(void)
{ int rep_ack;
	uint8_t bits_entete;
	uint16_t etat_ZIF16;
	init_proc();
	
	fifo_push_0x55_burst(256);   // on remplit une première fois//============================================================================================test

	while(1)
		{	
		if (flagMajCasiers) {flagMajCasiers = false;
				// Appliquer consigne_leds_vertes et consigne_leds_rouges
				//en envoyant les bonnes trames I2C, chaque PCF sera configur� avec une adresse diff�rente
			                  maj_leds_casiers(); // par ecriture i2C
			                  lire_etat_casiers(); // par lecture i2c
			                  envoyer_trame_casiers();
		}

		if (flagMajZIF16) {flagMajZIF16 = false;
				// Appliquer consigne_dir et consigne_etat
			                 init_ZIF16();//pour mettre � jour les pattes
			                 etat_ZIF16 = lire_etat_ZIF16();
			                 envoyer_trame_ZIF16();
											}	
	if(USART2->SR & USART_SR_RXNE )
	  { gere_serial2();// gerer la r�ception  serie
	  } //
	if (USART2->SR & USART_SR_TXE) {    // dépiler la FIFO si non vide
    if (place_libre < (int)sizeof(fifo)) {
        USART2->DR = fifo[pr];
        pr = (uint8_t)(pr + 1);              // taille 256 -> overflow naturel
        place_libre++;
    }
}
	if(INT_ADXL)
	{read_ADXL_sensors(&capteurs.b[0]);
	 envoi_trame_accelero();
    }
     		if (place_libre >= 32) { //============================================================================================test

    fifo_push_0x55_burst(32);
}
																	
 } // fin while(1)
}//fin main
