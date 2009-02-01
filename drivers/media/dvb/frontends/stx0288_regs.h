#ifndef H_STX0288_DRV
	#define H_STX0288_DRV

/* Register map constants */

/* ID */
 #define R288_ID 0x0
 #define F288_CHIP_IDENT 0xf0
 #define F288_RELEASE 0xf

/* I2CRPT */
 #define R288_I2CRPT 0x1
 #define F288_I2CT_ON 0x10080
 #define F288_ENARPT_LEVEL 0x10070
 #define F288_SCLT_DELAY 0x10008
 #define F288_SCLT_VALUE 0x10004
 #define F288_STOP_ENABLE 0x10002
 #define F288_SDAT_VALUE 0x10001

/* ACR */
 #define R288_ACR 0x2
 #define F288_PRESCALER 0x200e0
 #define F288_DIVIDER 0x2001f

/* F22FR */
 #define R288_F22FR 0x3
 #define F288_FFR_REG 0x300ff

/* F22RX */
 #define R288_F22RX 0x4
 #define F288_FRX_REG 0x400ff

/* DISEQC */
 #define R288_DISEQC 0x5
 #define F288_DIS_RESET 0x50080
 #define F288_TIM_OFF 0x50040
 #define F288_TIM_CMD 0x50038
 #define F288_DIS_PRECHARGE 0x50004
 #define F288_DISEQC_MODE 0x50003

/* DISEQCFIFO */
 #define R288_DISEQCFIFO 0x6
 #define F288_DISEQC_FIFO1 0x600ff

/* DISEQCSTAT1 */
 #define R288_DISEQCSTAT1 0x7
 #define F288_TX_FAIL 0x70080
 #define F288_FIFO_FULL 0x70040
 #define F288_TX_IDDLE 0x70020
 #define F288_READ_WRITE_COUNTER 0x7001f

/* DISEQCSTAT2 */
 #define R288_DISEQCSTAT2 0x8
 #define F288_GAPBURST_FLAG 0x80001

/* DISEQC2 */
 #define R288_DISEQC2 0x9
 #define F288_RECEIVER_ON 0x90080
 #define F288_IGNORE_SH22KHZ 0x90040
 #define F288_ONECHIP_TRX 0x90020
 #define F288_EXT_ENVEL 0x90010
 #define F288_PIN_SELECT 0x9000c
 #define F288_IRQ_RXEND 0x90002
 #define F288_IRQ_HALF_FIFO 0x90001

/* DISRX_ST0 */
 #define R288_DISRX_ST0 0xa
 #define F288_RX_END 0xa0080
 #define F288_RX_ACTIVE 0xa0040
 #define F288_SHORT_22KHZ 0xa0020
 #define F288_RX_COUNT_TONE 0xa0010
 #define F288_FIFO_8B_READY 0xa0008
 #define F288_FIFO_EMPTY 0xa0004
 #define F288_RX_NON_BYTE 0xa0002
 #define F288_ABORT_DISEQC 0xa0001

/* DISRX_ST1 */
 #define R288_DISRX_ST1 0xb
 #define F288_RX_FAIL 0xb0080
 #define F288_PARITY_FAIL 0xb0040
 #define F288_FIFO_OVERFLOW 0xb0020
 #define F288_FIFO_BYTENBR 0xb001f

/* DISTXWAIT */
 #define R288_DISTXWAIT 0xc
 #define F288_DISTXWAIT 0xc00ff

/* TSREG */
 #define R288_TSREG 0xd
 #define F288_ITSTATUS_MODE 0xd0008
 #define F288_DEMOUNT_M 0xd0004
 #define F288_SERIAL_OUT_D0 0xd0002
 #define F288_OUTRS_HZ 0xd0001

/* AGC1C */
 #define R288_AGC1C 0xe
 #define F288_ENA_DCADJ 0xe0080
 #define F288_AVERAGE_ON 0xe0040
 #define F288_AGC_OPDRAIN 0xe0020
 #define F288_IAGC 0xe0010
 #define F288_AGCIQ_BETA 0xe000f

/* AGC1R */
 #define R288_AGC1R 0xf
 #define F288_AGC1R_REF 0xf00ff

/* AGC1IN */
 #define R288_AGC1IN 0x10
 #define F288_AGC1_VALUE 0x1001ff

/* RTC */
 #define R288_RTC 0x11
 #define F288_ALPHA_TMG 0x1100f0
 #define F288_BETA_TMG 0x11000f

/* AGC2C0 */
 #define R288_AGC2C0 0x12
 #define F288_AGC2COEFF 0x120007

/* AGC2O */
 #define R288_AGC2O 0x13
 #define F288_AGC2_REF 0x13007f

/* STEP1 */
 #define R288_STEP1 0x14
 #define F288_STEP_MINUS 0x1400f0
 #define F288_STEP_PLUS 0x14000f

/* CFD */
 #define R288_CFD 0x15
 #define F288_CFD_ON 0x150080
 #define F288_BETA_FC 0x150070
 #define F288_FDCT 0x15000c
 #define F288_LDL 0x150003

/* ACLC */
 #define R288_ACLC 0x16
 #define F288_DEROT_ON_OFF 0x160080
 #define F288_ACLC 0x160040
 #define F288_NOISE 0x160030
 #define F288_ALPHA 0x16000f

/* BCLC */
 #define R288_BCLC 0x17
 #define F288_ALGO 0x1700c0
 #define F288_BETA 0x17003f

/* R8PSK */
 #define R288_R8PSK 0x18
 #define F288_MODE_COEF 0x180008

/* LDT */
 #define R288_LDT 0x19
 #define F288_LOCK_THRESHOLD 0x1901ff

/* LDT2 */
 #define R288_LDT2 0x1a
 #define F288_LOCK_THRESHOLD2 0x1a01ff

/* DACR1 */
 #define R288_DACR1 0x1b
 #define F288_DAC_MODE 0x1b00e0
 #define F288_DACI_LSB 0x1b000f

/* DACR2 */
 #define R288_DACR2 0x1c
 #define F288_DACI_MSB 0x1c00ff

/* TLIRM */
 #define R288_TLIRM 0x1e
 #define F288_TMG_LOCK 0x1e0080
 #define F288_TMG_FINAL_IND_MSB 0x1e001f

/* TLIRL */
 #define R288_TLIRL 0x1f
 #define F288_TMG_FINAL_IND_LSB 0x1f00ff

/* AGC2I1 */
 #define R288_AGC2I1 0x20
 #define F288_AGC2_INTEGRATOR_MSB 0x2000ff

/* AGC2I2 */
 #define R288_AGC2I2 0x21
 #define F288_AGC2_INTEGRATOR_LSB 0x2100ff

/* RTFM */
 #define R288_RTFM 0x22
 #define F288_TIMING_LOOP_FREQ_MSB 0x2200ff

/* RTFL */
 #define R288_RTFL 0x23
 #define F288_TIMING_LOOP_FREQ_LSB 0x2300ff

/* VSTATUS */
 #define R288_VSTATUS 0x24
 #define F288_CF 0x240080
 #define F288_VSTATUS_6 0x240040
 #define F288_VSTATUS_5 0x240020
 #define F288_PRF 0x240010
 #define F288_LK 0x240008
 #define F288_PR 0x240007

/* LDI */
 #define R288_LDI 0x25
 #define F288_LOCK_DET_INTEGR 0x2501ff

/* ECNTM */
 #define R288_ECNTM 0x26
 #define F288_ERROR_COUNT_MSB 0x2600ff

/* ECNTL */
 #define R288_ECNTL 0x27
 #define F288_ERROR_COUNT_LSB 0x2700ff

/* SFRH */
 #define R288_SFRH 0x28
 #define F288_SYMB_FREQ_HSB 0x2800ff

/* SFRM */
 #define R288_SFRM 0x29
 #define F288_SYMB_FREQ_MSB 0x2900ff

/* SFRL */
 #define R288_SFRL 0x2a
 #define F288_SYMB_FREQ_LSB 0x2a00f0

/* CFRM */
 #define R288_CFRM 0x2b
 #define F288_CARRIER_FREQUENCY_MSB 0x2b00ff

/* CFRL */
 #define R288_CFRL 0x2c
 #define F288_CARRIER_FREQUENCY_LSB 0x2c00ff

/* NIRM */
 #define R288_NIRM 0x2d
 #define F288_NOISE_IND_MSB 0x2d00ff

/* NIRL */
 #define R288_NIRL 0x2e
 #define F288_NOISE_IND_LSB 0x2e00ff

/* VERROR */
 #define R288_VERROR 0x2f
 #define F288_ERROR_VAL 0x2f00ff

/* FECM */
 #define R288_FECM 0x30
 #define F288_FECMODE 0x3000f0
 #define F288_FECM3 0x300008
 #define F288_VIT_DIFF 0x300004
 #define F288_SYNC 0x300002
 #define F288_SYM 0x300001

/* VTH0 */
 #define R288_VTH0 0x31
 #define F288_VTH0 0x31007f

/* VTH1 */
 #define R288_VTH1 0x32
 #define F288_VTH1 0x32007f

/* VTH2 */
 #define R288_VTH2 0x33
 #define F288_VTH2 0x33007f

/* VTH3 */
 #define R288_VTH3 0x34
 #define F288_VTH3 0x34007f

/* VTH4 */
 #define R288_VTH4 0x35
 #define F288_VTH4 0x35007f

/* VTH5 */
 #define R288_VTH5 0x36
 #define F288_VTH5 0x36007f

/* PR */
 #define R288_PR 0x37
 #define F288_E7 0x370080
 #define F288_E6 0x370040
 #define F288_PR_7_8 0x370020
 #define F288_PR_6_7 0x370010
 #define F288_PR_5_6 0x370008
 #define F288_PR_3_4 0x370004
 #define F288_PR_2_3 0x370002
 #define F288_PR_1_2 0x370001

/* VAVSRCH */
 #define R288_VAVSRCH 0x38
 #define F288_AM 0x380080
 #define F288_F 0x380040
 #define F288_SN 0x380030
 #define F288_TO 0x38000c
 #define F288_H 0x380003

/* RS */
 #define R288_RS 0x39
 #define F288_DEINT 0x390080
 #define F288_OUTRS_PS 0x390040
 #define F288_RS 0x390020
 #define F288_DESCRAM 0x390010
 #define F288_ERR_BIT 0x390008
 #define F288_MPEG 0x390004
 #define F288_CLK_POL 0x390002
 #define F288_CLK_CFG 0x390001

/* RSOUT */
 #define R288_RSOUT 0x3a
 #define F288_INV_DVALID 0x3a0080
 #define F288_INV_DSTART 0x3a0040
 #define F288_INV_DERROR 0x3a0020
 #define F288_EN_STBACKEND 0x3a0010
 #define F288_ENA8_LEVEL 0x3a000f

/* ERRCTRL */
 #define R288_ERRCTRL 0x3b
 #define F288_ERRMODE 0x3b0080
 #define F288_TSTERS 0x3b0040
 #define F288_ERR_SOURCE 0x3b0030
 #define F288_ECOL3 0x3b0008
 #define F288_RESET_CNT 0x3b0004
 #define F288_NOE 0x3b0003

/* VITPROG */
 #define R288_VITPROG 0x3c
 #define F288_VITPROG_7 0x3c0080
 #define F288_VITPROG_6 0x3c0040
 #define F288_VITPROG_5 0x3c0020
 #define F288_SWAP_ENABLE 0x3c0010
 #define F288_VITPROG_3 0x3c0008
 #define F288_VITPROG_2 0x3c0004
 #define F288_MDIVIDER 0x3c0003

/* ERRCTRL2 */
 #define R288_ERRCTRL2 0x3d
 #define F288_ERRMODE2 0x3d0080
 #define F288_TSTERS2 0x3d0040
 #define F288_ERR_SOURCE2 0x3d0030
 #define F288_ECOL3_2 0x3d0008
 #define F288_RESET_CNT2 0x3d0004
 #define F288_NOE2 0x3d0003

/* ECNTM2 */
 #define R288_ECNTM2 0x3e
 #define F288_ERROR_COUNT2_MSB 0x3e00ff

/* ECNTL2 */
 #define R288_ECNTL2 0x3f
 #define F288_ERROR_COUNT2_LSB 0x3f00ff

/* PLLCTRL */
 #define R288_PLLCTRL 0x40
 #define F288_PLL_MDIV 0x4000ff

/* SYNTCTRL */
 #define R288_SYNTCTRL 0x41
 #define F288_STANDBY 0x410080
 #define F288_PLL_STOP 0x410010
 #define F288_SEL_OSCI 0x410008
 #define F288_PLL_SELRATIO 0x410004
 #define F288_BYP_PLL_ADC 0x410002
 #define F288_BYPASS_PLL 0x410001

/* TSTTNR1 */
 #define R288_TSTTNR1 0x42
 #define F288_ADC_INVCLK 0x420040
 #define F288_ADC_PON 0x420020
 #define F288_ADC_INMODE 0x420010
 #define F288_OSCI_STOP_I2C 0x420002
 #define F288_REGPOFF 0x420001

/* IRQMSKM */
 #define R288_IRQMSKM 0x43
 #define F288_IRQ_MSK_MSB 0x4300ff

/* IRQMSKL */
 #define R288_IRQMSKL 0x44
 #define F288_IRQ_MSK_LSB 0x4400ff

/* IRQSTATM */
 #define R288_IRQSTATM 0x45
 #define F288_IRQ_STATUS_MSB 0x4500ff

/* IRQSTATL */
 #define R288_IRQSTATL 0x46
 #define F288_IRQ_STATUS_LSB 0x4600ff

/* IRQCFG */
 #define R288_IRQCFG 0x47
 #define F288_INV1 0x470080
 #define F288_CHOICE1 0x470070
 #define F288_INV2 0x470008
 #define F288_CHOICE2 0x470007

/* SYMBCTRL */
 #define R288_SYMBCTRL 0x4a
 #define F288_SYMB_CHOICE 0x4a0003

/* ISYMB */
 #define R288_ISYMB 0x4b
 #define F288_I_SYMB 0x4b01ff

/* QSYMB */
 #define R288_QSYMB 0x4c
 #define F288_Q_SYMB 0x4c01ff

/* ASCTRL */
 #define R288_ASCTRL 0x50
 #define F288_FROZE_LOCK 0x500020
 #define F288_KI 0x500018
 #define F288_AUTOCENTRE 0x500004
 #define F288_FINE 0x500002
 #define F288_COARSE 0x500001

/* COARP1 */
 #define R288_COARP1 0x51
 #define F288_KT 0x51007f

/* COARP2 */
 #define R288_COARP2 0x52
 #define F288_KC 0x520038
 #define F288_KS 0x520007

/* FMINM */
 #define R288_FMINM 0x53
 #define F288_STOP_ON_FMIN 0x530080
 #define F288_FMIN_MSB 0x53007f

/* FMINL */
 #define R288_FMINL 0x54
 #define F288_FMIN_LSB 0x5400ff

/* FMAXM */
 #define R288_FMAXM 0x55
 #define F288_STOP_ON_FMAX 0x550080
 #define F288_FMAX_MSB 0x55007f

/* FMAXL */
 #define R288_FMAXL 0x56
 #define F288_FMAX_LSB 0x5600ff

/* FINEINC */
 #define R288_FINEINC 0x57
 #define F288_FINE_INCR 0x5701ff

/* STEP2 */
 #define R288_STEP2 0x58
 #define F288_STEP2_MINUS 0x5800f0
 #define F288_STEP2_PLUS 0x58000f

/* TH2 */
 #define R288_TH2 0x59
 #define F288_TH2_MSB 0x5900ff

/* TH2_TH1 */
 #define R288_TH2_TH1 0x5a
 #define F288_TH2_LSB 0x5a00c0
 #define F288_TH1_MSB 0x5a0003

/* TH1 */
 #define R288_TH1 0x5b
 #define F288_TH1_LSB 0x5b00ff

/* THH */
 #define R288_THH 0x5c
 #define F288_THH 0x5c003f

/* IND1MAX */
 #define R288_IND1MAX 0x5d
 #define F288_IND1_TRESH 0x5d00ff

/* ACCU1VAL */
 #define R288_ACCU1VAL 0x5e
 #define F288_IND1_ACC 0x5e00ff

/* ACCU2VAL */
 #define R288_ACCU2VAL 0x5f
 #define F288_IND2_ACC 0x5f00ff

/* IOPGPIO0 */
 #define R288_IOPGPIO0 0x60
 #define F288_IOP0_HIGHZ 0x600080
 #define F288_IOP0_CFG 0x60007e
 #define F288_IOP0_XOR 0x600001

/* IOPGPIO1 */
 #define R288_IOPGPIO1 0x61
 #define F288_IOP1_HIGHZ 0x610080
 #define F288_IOP1_CFG 0x61007e
 #define F288_IOP1_XOR 0x610001

/* IOPGPIO2 */
 #define R288_IOPGPIO2 0x62
 #define F288_IOP2_HIGHZ 0x620080
 #define F288_IOP2_CFG 0x62007e
 #define F288_IOP2_XOR 0x620001

/* IOPGPIO3 */
 #define R288_IOPGPIO3 0x63
 #define F288_IOP3_HIGHZ 0x630080
 #define F288_IOP3_CFG 0x63007e
 #define F288_IOP3_XOR 0x630001

/* IOPGPIO4 */
 #define R288_IOPGPIO4 0x64
 #define F288_IOP4_HIGHZ 0x640080
 #define F288_IOP4_CFG 0x64007e
 #define F288_IOP4_XOR 0x640001

/* IOPGPIO5 */
 #define R288_IOPGPIO5 0x65
 #define F288_IOP5_HIGHZ 0x650080
 #define F288_IOP5_CFG 0x65007e
 #define F288_IOP5_XOR 0x650001

/* IOPGPIO6 */
 #define R288_IOPGPIO6 0x66
 #define F288_IOP6_HIGHZ 0x660080
 #define F288_IOP6_CFG 0x66007e
 #define F288_IOP6_XOR 0x660001

/* IOPGPIO7 */
 #define R288_IOPGPIO7 0x67
 #define F288_IOP7_HIGHZ 0x670080
 #define F288_IOP7_CFG 0x67007e
 #define F288_IOP7_XOR 0x670001

/* IOPGPIO8 */
 #define R288_IOPGPIO8 0x68
 #define F288_IOP8_HIGHZ 0x680080
 #define F288_IOP8_CFG 0x68007e
 #define F288_IOP8_XOR 0x680001

/* IOPGPIO9 */
 #define R288_IOPGPIO9 0x69
 #define F288_IOP9_HIGHZ 0x690080
 #define F288_IOP9_CFG 0x69007e
 #define F288_IOP9_XOR 0x690001

/* IOPVAL0 */
 #define R288_IOPVAL0 0x6a
 #define F288_GPIO7 0x6a0080
 #define F288_GPIO6 0x6a0040
 #define F288_GPIO5 0x6a0020
 #define F288_GPIO4 0x6a0010
 #define F288_GPIO3 0x6a0008
 #define F288_GPIO2 0x6a0004
 #define F288_GPIO1 0x6a0002
 #define F288_GPIO0 0x6a0001

/* IOPVAL1 */
 #define R288_IOPVAL1 0x6b
 #define F288_CS1 0x6b0080
 #define F288_CS0 0x6b0040
 #define F288_STDBY 0x6b0020
 #define F288_AUXCLK 0x6b0010
 #define F288_DIRCLK 0x6b0008
 #define F288_AGC 0x6b0004
 #define F288_GPIO9 0x6b0002
 #define F288_GPIO8 0x6b0001

/* IOPVAL2 */
 #define R288_IOPVAL2 0x6c
 #define F288_DISEQCOUT 0x6c0001

/* FREEDIS */
 #define R288_FREEDIS 0x70
 #define F288_DIV_XXX_FREE 0x7000ff

/* FREES */
 #define R288_FREES 0x71
 #define F288_SYSTEM_FREE1 0x7100ff

/* FREESA */
 #define R288_FREESA 0x72
 #define F288_SAMP_FREE 0x7200ff

/* FREEVIT */
 #define R288_FREEVIT 0x74
 #define F288_VITERBI_FREE 0x7400ff

/* FREERS */
 #define R288_FREERS 0x75
 #define F288_REED_SOLO_FREE55 0x7500ff

/* FREEQDM */
 #define R288_FREEQDM 0x76
 #define F288_DEMOD_FREE56 0x7600ff

/* TAGC1 */
 #define R288_TAGC1 0x81
 #define F288_SEL_ADC_PLL 0x810010
 #define F288_AGC_BEF_DC 0x810008
 #define F288_EN_AGC_PWM 0x810004
 #define F288_EN_AGC_PLF 0x810002
 #define F288_EN_AGC_TST 0x810001

/* IDCOFF */
 #define R288_IDCOFF 0x82
 #define F288_I_DC_OFFSET_VALUE 0x82003f

/* QDCOFF */
 #define R288_QDCOFF 0x83
 #define F288_Q_DC_OFFSET_VALUE 0x83003f

/* TSTR */
 #define R288_TSTR 0x84
 #define F288_EN_STRST 0x840080
 #define F288_EN_TMG_LOC2 0x840040
 #define F288_EN_TMG_LOC1 0x840020
 #define F288_EN_STR_FRAC 0x840010
 #define F288_EN_STR_ERR 0x840004
 #define F288_EN_STR_GPD 0x840002

/* TCTLT1 */
 #define R288_TCTLT1 0x85
 #define F288_SEL_COR 0x850080
 #define F288_NOISE_IND_CHOICE 0x85000c
 #define F288_SEL_TETA 0x850002
 #define F288_SEL_CTL_PLF 0x850001

/* TSTRM1 */
 #define R288_TSTRM1 0x88
 #define F288_SELOUTR1 0x880080
 #define F288_FSELRAM1 0x880040
 #define F288_FSELDEC 0x880020
 #define F288_FOEB 0x88001c
 #define F288_FADR 0x880003

/* TSTRATE */
 #define R288_TSTRATE 0x89
 #define F288_FORCEPHA 0x890080
 #define F288_TSTRATE6 0x890040
 #define F288_TSTRATE5 0x890020
 #define F288_FNEWALPHA 0x890010
 #define F288_FROT90 0x890008
 #define F288_FOFF 0x890004
 #define F288_FR1 0x890002
 #define F288_FR2 0x890001

/* SELOUT */
 #define R288_SELOUT 0x8a
 #define F288_EN_VLOG 0x8a0080
 #define F288_SELVIT60 0x8a0040
 #define F288_SELSYN3 0x8a0020
 #define F288_SELSYN2 0x8a0010
 #define F288_SELSYN1 0x8a0008
 #define F288_SELLIFO 0x8a0004
 #define F288_SELFIFO 0x8a0002
 #define F288_SELERR 0x8a0001

/* FORCEIN */
 #define R288_FORCEIN 0x8b
 #define F288_SELVITDATAIN 0x8b0080
 #define F288_FORCE_ACS 0x8b0040
 #define F288_TSTSYN 0x8b0020
 #define F288_TSTRAM64 0x8b0010
 #define F288_TSTRAM 0x8b0008
 #define F288_TSTERR2 0x8b0004
 #define F288_TSTERR1 0x8b0002
 #define F288_TSTACS 0x8b0001

/* TSTFIFOL */
 #define R288_TSTFIFOL 0x8c
 #define F288_TSTFIFO7 0x8c0080
 #define F288_TSTFIFO6 0x8c0040
 #define F288_TSTFIFO5 0x8c0020
 #define F288_TSTFIFO3 0x8c0008
 #define F288_FORMSB 0x8c0004
 #define F288_FORLSB 0x8c0002
 #define F288_TST_FIFO 0x8c0001

/* TSTCK */
 #define R288_TSTCK 0x90
 #define F288_TSTCKRST 0x900040
 #define F288_TSTCKDIL 0x900020
 #define F288_FORCERATE1 0x900008
 #define F288_FORCESYMHA 0x900004
 #define F288_FORSYMAX 0x900002
 #define F288_DIRCKINT 0x900001

/* TSTRES */
 #define R288_TSTRES 0x91
 #define F288_FRESYYY 0x910080
 #define F288_FREESRS 0x910040
 #define F288_FRESXXX 0x910020
 #define F288_FRESCAR 0x910010
 #define F288_FRESACS 0x910008
 #define F288_FRESYM 0x910004
 #define F288_FRESMAS 0x910002
 #define F288_FRESINT 0x910001

/* TSTOUT */
 #define R288_TSTOUT 0x92
 #define F288_RBACT 0x920040
 #define F288_TS 0x92000e
 #define F288_CTEST 0x920001

/* TSTIN */
 #define R288_TSTIN 0x93
 #define F288_TEST_IN 0x930080
 #define F288_EN_ADC 0x930040
 #define F288_SGN_ADC 0x930020
 #define F288_BCLK_IN 0x930010
 #define F288_TP12 0x930008

/* READREG */
 #define R288_READREG 0x94
 #define F288_READREG 0x9400ff

/* TSTNR2 */
 #define R288_TSTNR2 0x97
 #define F288_DISEQC_IDDQ 0x970020
 #define F288_DISEQC_I2C 0x97001f

/* TSTDIS */
 #define R288_TSTDIS 0xa0
 #define F288_EN_DIS 0xa00080
 #define F288_EN_PTRS 0xa00040
 #define F288_TST_DIS_5 0xa00020
 #define F288_TST_DIS_4 0xa00010
 #define F288_EN_DIS_FIFOS 0xa00008
 #define F288_TST_PRO 0xa00004
 #define F288_TST_REG 0xa00002
 #define F288_TST_PRE 0xa00001

/* TSTDISRX */
 #define R288_TSTDISRX 0xa1
 #define F288_EN_DISRX 0xa10080
 #define F288_TST_CUR_SRC 0xa10040
 #define F288_IN_DIGSIG 0xa10020
 #define F288_HIZ_CUR_SRC 0xa10010
 #define F288_PIN_SELECT_TST 0xa10008
 #define F288_TST_DISRX 0xa10007

/* IOPSDAT */
 #define R288_IOPSDAT 0xb0
 #define F288_SDAT_HIGHZ 0xb00080
 #define F288_SDAT_CFG 0xb0007e
 #define F288_SDAT_XOR 0xb00001

/* IOPSCLT */
 #define R288_IOPSCLT 0xb1
 #define F288_SCLT_HIGHZ 0xb10080
 #define F288_SCLT_CFG 0xb1007e
 #define F288_SCLT_XOR 0xb10001

/* IOPAGC */
 #define R288_IOPAGC 0xb2
 #define F288_AGC_HIGHZ 0xb20080
 #define F288_AGC_CFG 0xb2007e
 #define F288_AGC_XOR 0xb20001

/* IOPDIRCLK */
 #define R288_IOPDIRCLK 0xb3
 #define F288_DIRCLK_HIGHZ 0xb30080
 #define F288_DIRCLK_CFG 0xb3007e
 #define F288_DIRCLK_XOR 0xb30001

/* IOPAUX */
 #define R288_IOPAUX 0xb4
 #define F288_AUX_HIGHZ 0xb40080
 #define F288_AUX_CFG 0xb4007e
 #define F288_AUX_XOR 0xb40001

/* IOPSTDBY */
 #define R288_IOPSTDBY 0xb5
 #define F288_STDBY_HIGHZ 0xb50080
 #define F288_STDBY_CFG 0xb5007e
 #define F288_STDBY_XOR 0xb50001

/* IOPCS0 */
 #define R288_IOPCS0 0xb6
 #define F288_CS0_HIGHZ 0xb60080
 #define F288_CS0_CFG 0xb6007e
 #define F288_CS0_XOR 0xb60001

/* IOPCS1 */
 #define R288_IOPCS1 0xb7
 #define F288_CS1_HIGHZ 0xb70080
 #define F288_CS1_CFG 0xb7007e
 #define F288_CS1_XOR 0xb70001

/* IOPSDISEQC */
 #define R288_IOPSDISEQC 0xb8
 #define F288_DISEQC_HIGHZ 0xb80080
 #define F288_DISEQC_CFG 0xb8007e
 #define F288_DISEQC_XOR 0xb80001

/* TBUSBIT */
 #define R288_TBUSBIT 0xb9
 #define F288_BUS_CHOICE 0xb90040
 #define F288_BUS_POSITION 0xb9003f

/* TCOMP1 */
 #define R288_TCOMP1 0xf1
 #define F288_SLEEPINHBT 0xf10080
 #define F288_RA6SRC 0xf10040
 #define F288_RA5SRC 0xf10020
 #define F288_RA4SRC 0xf10010
 #define F288_RA3SRC 0xf10008
 #define F288_RA2SRC 0xf10004
 #define F288_RA1SRC 0xf10002
 #define F288_RA0SRC 0xf10001

/* TCOMP2 */
 #define R288_TCOMP2 0xf0
 #define F288_ACCURATE 0xf00010
 #define F288_COMPENS 0xf00008
 #define F288_COMPTQ 0xf00004
 #define F288_FREEZE 0xf00002
 #define F288_CHIPSLEEP 0xf00001

/* TCOMPSTAT */
 #define R288_TCOMPSTAT 0xf2
 #define F288COMPOK 0xf20080
 #define F288A6SRC 0xf20040
 #define F288A5SRC 0xf20020
 #define F288A4SRC 0xf20010
 #define F288A3SRC 0xf20008
 #define F288A2SRC 0xf20004
 #define F288A1SRC 0xf20002
 #define F288A0SRC 0xf20001

#endif

