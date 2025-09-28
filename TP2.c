#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>
#include "ADXL345.h"

#ifndef ADXL_DATAX0
#define ADXL_DATAX0     ADXL345_DATAX0
#endif
#ifndef ADXL_POWER_CTL
#define ADXL_POWER_CTL  ADXL345_POWER_CTL
#endif

/* ======= Types ADXL (sécurité si non définis dans l'en-tête) ======= */
#ifndef ADXL345_TYPES_DEFINED
#define ADXL345_TYPES_DEFINED
typedef uint8_t adresse_type;   /* adresse registre ADXL345 (6 bits utiles) */
typedef uint8_t val_type;       /* valeur 8 bits */
typedef uint8_t type_retour;    /* valeur de retour */
#endif

/* ======= Prototypes (C90) ======= */
static void CS_ADXL(uint8_t sel);

static void SDA_OUT(void);
static void SDA_IN(void);
static void SCL_OUT(void);
static void SDA_HI(void);
static void SDA_LO(void);
static void SCL_HI(void);
static void SCL_LO(void);
static uint8_t SDA_READ(void);
static void i2c_udelay(void);
static void I2C_start(void);
static void I2C_stop(void);
static uint8_t I2C_write_byte(uint8_t v);
static void init_I2C_BITBANGING(void);

static void I2C_write_PCF8574(uint8_t addr7, uint8_t data);

static uint8_t fifo_next(uint8_t idx);
static void fifo_push(uint8_t v);
static void fifo_push_0x55_burst(uint16_t n);

static uint32_t get_sysclk_hz(void);
static uint32_t get_hclk_hz(void);
static uint32_t get_pclk1_hz(void);
static void config_usart2(void);

static void init_SPI1(void);
static uint8_t spi1_txrx(uint8_t v);

static void write_ADXL_reg(adresse_type reg, val_type v);
static val_type read_ADXL_reg(adresse_type reg);
static void read_ADXL_sensors(uint8_t *buf6);

static void envoyer_trame_casiers(void);
void         maj_leds_casiers(void);
static void  lire_etat_casiers(void);

static void gere_serial2(void);

static void envoi_trame_accelero(void);

/* ======= Constantes/Macros ======= */
#define SELECT    0u
#define DESELECT  1u

/* I2C bit-bang sur PB8=SCL, PB9=SDA */
#define SCL_PIN   8u
#define SDA_PIN   9u

/* UART */
#define USART2_BAUD   9600u

/* FIFO TX */
#define FIFO_SZ  128

/* PCF8574 : mets l'adresse réelle selon A2/A1/A0 (0x20..0x27) */
#define PCF_LED_ADDR   0x20u

/* IRQ ADXL sur PB0 (exemple) : active bas */
#define INT_ADXL   ( ((GPIOB->IDR>>0)&1u) == 0u )

/* ======= Variables globales ======= */
static volatile uint8_t fifo[FIFO_SZ];
static volatile uint8_t pr=0, pw=0;
static volatile int     place_libre = FIFO_SZ;

/* consignes LED */
static volatile uint16_t consigne_leds_vertes = 0;
static volatile uint16_t consigne_leds_rouges = 0;

/* union 16 bits pour (re)construction mots depuis bytes */
typedef union { uint16_t w; uint8_t b[2]; } U16;

/* buffer accéléro */
typedef struct {
    uint8_t x0;
    uint8_t x1;
    uint8_t y0;
    uint8_t y1;
    uint8_t z0;
    uint8_t z1;
} U6_axes;

typedef union {
    U6_axes axes;
    uint8_t b[6];
} U6;
static U6 capteurs;

/* ======= Chip Select ADXL (PA4) ======= */
static void CS_ADXL(uint8_t sel)
{
    if(sel) GPIOA->BSRR = (1u<<4);    /* DESELECT (nCS=1) */
    else    GPIOA->BRR  = (1u<<4);    /* SELECT   (nCS=0) */
}

/* ======= I2C bit-banging ======= */
static void SDA_OUT(void) {
    uint32_t shift = (SDA_PIN-8u)*4u;
    GPIOB->CRH = (GPIOB->CRH & ~(0xFu << shift)) | (0x6u << shift); /* MODE=10 (2MHz), CNF=01 (OD) */
}
static void SDA_IN(void) {
    uint32_t shift = (SDA_PIN-8u)*4u;
    GPIOB->CRH = (GPIOB->CRH & ~(0xFu << shift)) | (0x4u << shift); /* Input floating */
}
static void SCL_OUT(void) {
    uint32_t shift = (SCL_PIN-8u)*4u;
    GPIOB->CRH = (GPIOB->CRH & ~(0xFu << shift)) | (0x6u << shift); /* MODE=10 (2MHz), CNF=01 (OD) */
}

static void SDA_HI(void){ GPIOB->BSRR = (1u<<SDA_PIN); }
static void SDA_LO(void){ GPIOB->BRR  = (1u<<SDA_PIN); }
static void SCL_HI(void){ GPIOB->BSRR = (1u<<SCL_PIN); }
static void SCL_LO(void){ GPIOB->BRR  = (1u<<SCL_PIN); }
static uint8_t SDA_READ(void){ return (GPIOB->IDR >> SDA_PIN) & 1u; }

/* Petit délai I2C */
static void i2c_udelay(void){ __NOP(); __NOP(); __NOP(); __NOP(); }

static void I2C_start(void)
{
    SDA_OUT(); SCL_OUT();
    SDA_HI(); SCL_HI(); i2c_udelay();
    SDA_LO(); i2c_udelay();
    SCL_LO(); i2c_udelay();
}
static void I2C_stop(void)
{
    SDA_OUT();
    SDA_LO(); SCL_HI(); i2c_udelay();
    SDA_HI(); i2c_udelay();
}
static uint8_t I2C_write_byte(uint8_t v)
{
    int i;
    SDA_OUT();
    for(i=7;i>=0;i--){
        if(v & (1u<<i)) SDA_HI(); else SDA_LO();
        i2c_udelay();
        SCL_HI(); i2c_udelay();
        SCL_LO(); i2c_udelay();
    }
    /* ACK */
    SDA_IN(); i2c_udelay();
    SCL_HI(); i2c_udelay();
    {
        uint8_t ack = (SDA_READ()==0u);
        SCL_LO(); i2c_udelay();
        SDA_OUT();
        return ack;
    }
}

static void init_I2C_BITBANGING(void)
{
    /* Horloge GPIOB */
    RCC->APB2ENR |= (1<<3); /* IOPBEN */
    /* Pull-ups externes requis (≈4.7 kΩ) sur PB8/PB9 ! */
    SCL_OUT(); SDA_OUT();
    SCL_HI(); SDA_HI();
}

/* ======= PCF8574 ======= */
static void I2C_write_PCF8574(uint8_t addr7, uint8_t data)
{
    I2C_start();
    (void)I2C_write_byte( (uint8_t)((addr7<<1) | 0u) ); /* W */
    (void)I2C_write_byte( data );
    I2C_stop();
}

/* ======= FIFO TX ======= */
static uint8_t fifo_next(uint8_t idx)
{
    idx++;
    if (idx >= FIFO_SZ) {
        idx = 0;
    }
    return idx;
}

static void fifo_push(uint8_t v)
{
    if(place_libre>0){
        fifo[pw] = v;
        pw = fifo_next(pw);
        place_libre--;
    }
}

static void fifo_push_0x55_burst(uint16_t n)
{
    while(n--){
        fifo_push(0x55);
    }
}

/* ======= Clocks pour UART ======= */
static uint32_t get_sysclk_hz(void) {
    uint32_t cfgr = RCC->CFGR;
    uint32_t sws  = (cfgr >> 2) & 0x3;  /* SWS */

    if (sws == 0) {                     /* HSI */
        return 8000000UL;
    } else if (sws == 1) {              /* HSE (supposé 8 MHz) */
        return 8000000UL;
    } else {                            /* PLL */
        uint32_t pllmul = ((cfgr >> 18) & 0xF) + 2U;   /* x2..x16 */
        uint32_t pllsrc = (cfgr >> 16) & 1U;           /* 0: HSI/2, 1: HSE */
        uint32_t pllclk_in = (pllsrc ? 8000000UL : 4000000UL);
        return pllclk_in * pllmul;
    }
}
static uint32_t get_hclk_hz(void) {
    static const uint16_t ahb_presc[16]  = {1,1,1,1, 1,1,1,1, 2,4,8,16, 64,128,256,512};
    uint32_t cfgr = RCC->CFGR;
    uint32_t hpre = (cfgr >> 4) & 0xF;
    uint32_t sys  = get_sysclk_hz();
    return sys / ahb_presc[hpre];
}
static uint32_t get_pclk1_hz(void) {
    static const uint8_t apb_presc[8] = {1,1,1,1, 2,4,8,16};
    uint32_t cfgr  = RCC->CFGR;
    uint32_t ppre1 = (cfgr >> 8) & 0x7;
    uint32_t hclk  = get_hclk_hz();
    return hclk / apb_presc[ppre1];
}

/* ======= USART2 ======= */
static void config_usart2(void)
{
    uint32_t pclk1, div, mantissa, fraction;

    /* GPIOA + AFIO */
    RCC->APB2ENR |= (1<<2) | (1<<0);   /* IOPAEN, AFIOEN */

    /* USART2 clock */
    RCC->APB1ENR |= (1<<17);           /* USART2EN */

    /* PA2/PA3: PA2=AF PP 2MHz, PA3=Input floating */
    GPIOA->CRL &= ~((0xFu<<8) | (0xFu<<12));
    GPIOA->CRL |=  (0xAu<<8) | (0x4u<<12);

    /* Pas de remap (PA2/PA3) */
    AFIO->MAPR &= ~(1<<3);

    /* Reset config */
    USART2->CR1 = 0; USART2->CR2 = 0; USART2->CR3 = 0;

    /* BRR depuis PCLK1 réel (oversampling x16) */
    pclk1   = get_pclk1_hz();
    div     = (pclk1 + (USART2_BAUD/2U)) / USART2_BAUD; /* arrondi */
    mantissa= div / 16U;
    fraction= div - (mantissa * 16U);
    USART2->BRR = (mantissa << 4) | (fraction & 0xFu);

    /* 8N1, TE|RE, UE */
    USART2->CR2 &= ~0x3000;           /* 1 stop */
    USART2->CR1 &= ~(0x1000 | 0x0400);/* 8 bits, pas de parité */
    USART2->CR1 |=  0x0008 | 0x0004;  /* TE|RE */
    USART2->CR1 |=  0x2000;           /* UE */

    /* Purge flags pendants */
    (void)USART2->SR; (void)USART2->DR;
}

/* ======= SPI1 ======= */
static void init_SPI1(void)
{
    /* GPIOA + SPI1 clocks */
    RCC->APB2ENR |= (1<<2) | (1<<12); /* IOPAEN + SPI1EN */

    /* PA5=SCK, PA6=MISO, PA7=MOSI, PA4=nCS */
    GPIOA->CRL &= ~((0xFu<<(5*4)) | (0xFu<<(6*4)) | (0xFu<<(7*4)) | (0xFu<<(4*4)));
    GPIOA->CRL |=  ((0x9u<<(5*4)) | (0x4u<<(6*4)) | (0x9u<<(7*4)) | (0x1u<<(4*4)));
    /* SCK/MOSI: AF PP 10MHz (0x9), MISO: input floating (0x4), nCS: out PP 10MHz (0x1) */

    /* Master, BR=F_PCLK/8, CPOL=0, CPHA=0, MSB first, SSM+SSI */
    SPI1->CR1 = (1<<2) | (0x2<<3) | (1<<9) | (1<<8);
    SPI1->CR1 |= (1<<6); /* SPE */

    /* nCS inactif */
    CS_ADXL(DESELECT);
}

static uint8_t spi1_txrx(uint8_t v)
{
    while((SPI1->SR & (1<<1)) == 0u) { /* TXE */ }
    *(volatile uint8_t *)&SPI1->DR = v;
    while((SPI1->SR & (1<<0)) == 0u) { /* RXNE */ }
    return *(volatile uint8_t *)&SPI1->DR;
}

/* ======= ADXL345 (exemples) ======= */
static void write_ADXL_reg(adresse_type reg, val_type v)
{
    CS_ADXL(SELECT);
    spi1_txrx(0x0A);       /* WRITE, MB=0 */
    spi1_txrx(reg);
    spi1_txrx(v);
    CS_ADXL(DESELECT);
}

static val_type read_ADXL_reg(adresse_type reg)
{
    val_type v;
    CS_ADXL(SELECT);
    spi1_txrx(0x0B);       /* READ, MB=0 */
    spi1_txrx(reg);
    v = spi1_txrx(0x00);
    CS_ADXL(DESELECT);
    return v;
}

static void read_ADXL_sensors(uint8_t *buf6)
{
    int i;
    CS_ADXL(SELECT);
    spi1_txrx(0x0B | (1<<6)); /* READ + MB=1 */
    spi1_txrx(ADXL_DATAX0);
    for(i=0;i<6;i++) buf6[i] = spi1_txrx(0x00);
    CS_ADXL(DESELECT);
}

/* ======= CASIERS (LEDs + retour) ======= */
static void envoyer_trame_casiers(void)
{
    /* Réponse 0xC0 + 2 data (état capteurs) — ici 0 pour l’exemple */
    uint8_t d0 = 0, d1 = 0;
    uint8_t hdr = 0xC0
                | ((d0>>7)&1u)
                | (((d1>>7)&1u)<<1);
    fifo_push(hdr);
    fifo_push((uint8_t)(d0 & 0x7Fu));
    fifo_push((uint8_t)(d1 & 0x7Fu));
}

/* Un seul PCF: P0 = LED verte, P1 = LED rouge (actif bas) */
void maj_leds_casiers(void)
{
    uint8_t bV = (consigne_leds_vertes & 1u) ? 1u : 0u; /* P0 = vert */
    uint8_t bR = (consigne_leds_rouges & 1u) ? 1u : 0u; /* P1 = rouge */

    /* Avant inversion: 1 = LED ON → après ~ : 0 (actif bas sur PCF) */
    /* P2..P7 restent 0 avant inversion → 1 après (~) = inactifs */
    {
        uint8_t pre  = (uint8_t)((bV<<0) | (bR<<1));
        uint8_t data = (uint8_t)(~pre);
        I2C_write_PCF8574(PCF_LED_ADDR, data);
    }
}

static void lire_etat_casiers(void)
{
    /* À implémenter si tu lis des entrées sur d’autres PCF, puis appeler envoyer_trame_casiers() */
}

/* ======= RX UART2 : parse trame “casiers” ======= */
static volatile uint8_t flagMajCasiers = 0;

static void gere_serial2(void)
{
    static uint8_t etat = 0;
    static uint8_t hdr  = 0;
    static U16 v, r;

    while( (USART2->SR & (1<<5)) != 0u )  /* RXNE */
    {
        uint8_t c = (uint8_t)USART2->DR;

        switch(etat){
        case 0: /* attendre entête (b7=1) */
            if(c & 0x80u){
                hdr  = c;
                etat = 1;
            }
            break;

        case 1: /* Verts MSB (b7=0) */
            v.b[1] = (uint8_t)((c & 0x7Fu) | ((hdr & 0x01u) ? 0x80u : 0x00u));
            etat = 2;
            break;

        case 2: /* Verts LSB (b7=0) */
            v.b[0] = (uint8_t)((c & 0x7Fu) | ((hdr & 0x02u) ? 0x80u : 0x00u));
            etat = 3;
            break;

        case 3: /* Rouges MSB (b7=0) */
            r.b[1] = (uint8_t)((c & 0x7Fu) | ((hdr & 0x04u) ? 0x80u : 0x00u));
            etat = 4;
            break;

        case 4: /* Rouges LSB (b7=0) */
            r.b[0] = (uint8_t)((c & 0x7Fu) | ((hdr & 0x08u) ? 0x80u : 0x00u));

            /* OK -> maj consignes + flag */
            consigne_leds_vertes = v.w;
            consigne_leds_rouges = r.w;
            flagMajCasiers = 1;

            etat = 0;
            break;

        default:
            etat = 0;
            break;
        }
    }
}

/* ======= Trame accéléro (exemple) ======= */
static void envoi_trame_accelero(void)
{
    int i;
    fifo_push(0x80);
    for(i=0;i<6;i++) fifo_push(capteurs.b[i]);
}

/* ======= MAIN ======= */
int main(void)
{
    /* Horloges GPIO A/B/C */
    RCC->APB2ENR |= (1<<2) | (1<<3) | (1<<4); /* IOPA, IOPB, IOPC */

    /* LED user PC13 (bluepill). Sur Nucleo, LED user est PA5 si besoin. */
    GPIOC->CRH = (GPIOC->CRH & ~(0xFu<<20)) | (0x1u<<20); /* PC13 out 10MHz PP */
    GPIOC->BSRR = (1<<13); /* LED off */

    /* Inits */
    init_I2C_BITBANGING();
    config_usart2();
    init_SPI1();

    /* ADXL minimal */
    write_ADXL_reg(ADXL_POWER_CTL, 0x08); /* Measure=1 */

    /* Debug option au boot */
    /* fifo_push_0x55_burst(64); */

    while(1)
    {
        /* RX UART */
        gere_serial2();

        /* Nouvelles consignes reçues ? */
        if(flagMajCasiers){
            flagMajCasiers = 0;
            maj_leds_casiers();
            /* éventuel écho d’état :
               lire_etat_casiers();
               envoyer_trame_casiers(); */
        }

        /* TX UART : dépile FIFO si TXE=1 */
        if ( (USART2->SR & (1<<7)) != 0u ) { /* TXE */
            if (place_libre < (int)sizeof(fifo)) {
                USART2->DR = fifo[pr];
                pr = fifo_next(pr);
                place_libre++;
            }
        }

        /* Accéléro : si IRQ active, lire et envoyer (exemple) */
        if (INT_ADXL) {
            read_ADXL_sensors(&capteurs.b[0]);
            envoi_trame_accelero();
        }

        /* Debug périodique :
           if (place_libre >= 32) fifo_push_0x55_burst(32); */
    }
}
/* ======= Fin ======= */
