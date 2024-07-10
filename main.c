#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

//i2c za led
I2C_HandleTypeDef hi2c1;
//spi za bmp280
SPI_HandleTypeDef hspi2;
//timer za debouncing, tipkalo za mjenjanje lcd zaslona
TIM_HandleTypeDef htim6;
//timer za debouncing, tipkalo za iskljuciti buzzer i ledice
TIM_HandleTypeDef htim7;
//timer za pwm buzzer
TIM_HandleTypeDef htim1;
//timer za treperenje ledica
TIM_HandleTypeDef htim10;

//true - temperatura, false - tlak
bool lcd_zaslon_temperatura_tlak = true;
//true - buzzer i ledice ne bi trebale raditi, false - mogu raditi
bool iskljuci_buzzer_ledice = false;

/////////////////////////////////////////////////////////////////////////////////////
//LCD
#define LCD_I2C_ADRESA 0x4E

void lcd_slanje_komande(char komanda) {
	char gornji_bajt, donji_bajt;
	uint8_t niz_podataka[4];

	// Odvajanje gornjih i donjih 4 bita komande
	gornji_bajt = (komanda & 0xF0);
	donji_bajt = ((komanda << 4) & 0xF0);

	// Priprema četiri podkomande koje će se slati
	niz_podataka[0] = gornji_bajt | 0x0C;  // en=1, rs=0
	niz_podataka[1] = gornji_bajt | 0x08;  // en=0, rs=0
	niz_podataka[2] = donji_bajt | 0x0C;   // en=1, rs=0
	niz_podataka[3] = donji_bajt | 0x08;   // en=0, rs=0

	// Slanje podkomandi putem I2C na LCD
	HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADRESA, (uint8_t*) niz_podataka, 4,
			100);
}

void lcd_slanje_podatka(char podatak) {
	char gornji_bajt, donji_bajt;
	uint8_t niz_podataka[4];

	// Odvajanje gornjih i donjih 4 bita podatka
	gornji_bajt = (podatak & 0xF0);
	donji_bajt = ((podatak << 4) & 0xF0);

	// Priprema četiri podkomande koje će se slati
	niz_podataka[0] = gornji_bajt | 0x0D;  // en=1, rs=0
	niz_podataka[1] = gornji_bajt | 0x09;  // en=0, rs=0
	niz_podataka[2] = donji_bajt | 0x0D;   // en=1, rs=0
	niz_podataka[3] = donji_bajt | 0x09;   // en=0, rs=0

	// Slanje podkomandi putem I2C na LCD
	HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADRESA, (uint8_t*) niz_podataka, 4,
			100);
}

void lcd_inicijalizacija(void) {
	// 4-bit inicijalizacija
	HAL_Delay(50);  // čekaj >40ms
	lcd_slanje_komande(0x30);
	HAL_Delay(5);   // čekaj >4.1ms
	lcd_slanje_komande(0x30);
	HAL_Delay(1);   // čekaj >100us
	lcd_slanje_komande(0x30);
	HAL_Delay(10);
	lcd_slanje_komande(0x20);  // 4-bit mode
	HAL_Delay(10);

	// inicijalizacija ekrana
	lcd_slanje_komande(0x28); // Funkcijski set --> DL=0 (4-bit mode), N = 1 (2-line display), F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_slanje_komande(0x08); // Upravljanje prikazom --> D=0, C=0, B=0  ---> isključi prikaz
	HAL_Delay(1);
	lcd_slanje_komande(0x01);  // Čišćenje ekrana
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_slanje_komande(0x06); // Postavljanje smjera unosa --> I/D = 1 (inkrementiranje kursora) & S = 0 (bez pomicanja)
	HAL_Delay(1);
	lcd_slanje_komande(0x0C); // Upravljanje prikazom --> D = 1, C i B = 0. (Kursor i blinkanje, zadnja dva bita)

}

void lcd_redak_stupac(int redak, int stupac) {
	switch (redak) {
	case 0:
		stupac |= 0x80; // Dodaj offset za prvi redak
		break;
	case 1:
		stupac |= 0xC0; // Dodaj offset za drugi redak
		break;
	}

	// Poziv funkcije za slanje komande postavljanja kursora
	lcd_slanje_komande(stupac);
}

void lcd_posalji_podatke(const char *format, ...) {
	va_list argumenti;
	va_start(argumenti, format);

	char niz_buffera[17]; // Prilagodite veličinu buffer-a prema potrebi
	vsnprintf(niz_buffera, sizeof(niz_buffera), format, argumenti);

	va_end(argumenti);

	char *pokazivac = niz_buffera; // Koristi pokazivač za navigaciju kroz niz

	while (*pokazivac) {
		lcd_slanje_podatka(*pokazivac++);
	}
}

void lcd_prazan_zaslon(void) {
	// Poziv funkcije za slanje komande za čišćenje ekrana
	lcd_slanje_komande(0x01);
	HAL_Delay(200);
}
//LCD
/////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//BMP280
//kalibracijske vrijednosti izvučene iz BMP280 da se dobije tocna temperatura i tlak
int32_t t_fine;
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

//inicijalizacija BMP280
bool BMP280_inicijalizacija() {

	char naredba[18] = { 0 };

	HAL_StatusTypeDef status1 = HAL_ERROR;
	HAL_StatusTypeDef status2 = HAL_ERROR;

	//slanje naredbe za adresu senzora, samo da provjerimo jel dobra
	naredba[0] = 0xD0 | 0x80;
	//spusti CS pin
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	//posalji naredbu
	status1 = HAL_SPI_Transmit(&hspi2, (uint8_t*) naredba, 1, HAL_MAX_DELAY);
	//pricekaj odgovor(adresu senzora u ovom slucaju)
	status2 = HAL_SPI_Receive(&hspi2, (uint8_t*) naredba, 1, HAL_MAX_DELAY);
	//digni CS pin, gotova komunikacija
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	if (status1 != HAL_OK || status2 != HAL_OK || naredba[0] != 0x58) {
		return false;
	};

	//tu pa nadalje saljemo neke naredbe za konfiguraciju senzora

	naredba[0] = 0xF4 & 0x7F; // ctrl_meas
	naredba[1] = (3 << 5) | (3 << 2) | 3; // Temperature oversampling x4, Pressure oversampling x4, Normal mode
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	status1 = HAL_SPI_Transmit(&hspi2, (uint8_t*) &naredba[0], 1,
	HAL_MAX_DELAY);
	status2 = HAL_SPI_Transmit(&hspi2, (uint8_t*) &naredba[1], 1,
	HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	if (status1 != HAL_OK || status2 != HAL_OK) {
		return false;
	};

	naredba[0] = 0xF5 & 0x7F; // config
	naredba[1] = (5 << 5) | (0 << 2) | 0; // Standby 1000ms, Filter off
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	status1 = HAL_SPI_Transmit(&hspi2, (uint8_t*) &naredba[0], 1,
	HAL_MAX_DELAY);
	status2 = HAL_SPI_Transmit(&hspi2, (uint8_t*) &naredba[1], 1,
	HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	if (status1 != HAL_OK || status2 != HAL_OK) {
		return false;
	};

	//tu pa nadalje citamo kalibracijske vrijednosti iz senzora

	naredba[0] = 0x88 | 0x80; // read dig_T regs
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	status1 = HAL_SPI_Transmit(&hspi2, (uint8_t*) naredba, 1, HAL_MAX_DELAY);
	status2 = HAL_SPI_Receive(&hspi2, (uint8_t*) naredba, 6, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	dig_T1 = (naredba[1] << 8) | naredba[0];
	dig_T2 = (naredba[3] << 8) | naredba[2];
	dig_T3 = (naredba[5] << 8) | naredba[4];

	if (status1 != HAL_OK || status2 != HAL_OK) {
		return false;
	};

	naredba[0] = 0x8E | 0x80; // read dig_P regs
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	status1 = HAL_SPI_Transmit(&hspi2, (uint8_t*) naredba, 1, HAL_MAX_DELAY);
	status2 = HAL_SPI_Receive(&hspi2, (uint8_t*) naredba, 18, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	if (status1 != HAL_OK || status2 != HAL_OK) {
		return false;
	};

	dig_P1 = (naredba[1] << 8) | naredba[0];
	dig_P2 = (naredba[3] << 8) | naredba[2];
	dig_P3 = (naredba[5] << 8) | naredba[4];
	dig_P4 = (naredba[7] << 8) | naredba[6];
	dig_P5 = (naredba[9] << 8) | naredba[8];
	dig_P6 = (naredba[11] << 8) | naredba[10];
	dig_P7 = (naredba[13] << 8) | naredba[12];
	dig_P8 = (naredba[15] << 8) | naredba[14];
	dig_P9 = (naredba[17] << 8) | naredba[16];

	return true;
}

//salje se naredba da se dobi raw value te se uz pomoc kalibriranih vrijednosti
//dobiva prava vrijednost temperature
float BMP280_dobij_temperaturu() {
	int32_t temperatura_raw, var1, var2, temperatura;
	float temperatura_float;
	char naredba[3];

	HAL_StatusTypeDef status1 = HAL_ERROR;
	HAL_StatusTypeDef status2 = HAL_ERROR;

	naredba[0] = 0xFA | 0x80;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	status1 = HAL_SPI_Transmit(&hspi2, (uint8_t*) naredba, 1, 100);
	status2 = HAL_SPI_Receive(&hspi2, (uint8_t*) naredba, 3, 100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	if (status1 != HAL_OK || status2 != HAL_OK) {
		return 0;
	};

	temperatura_raw = (naredba[0] << 12) | (naredba[1] << 4)
			| (naredba[2] >> 4);

	var1 = ((((temperatura_raw >> 3) - (dig_T1 << 1))) * dig_T2) >> 11;
	var2 = (((((temperatura_raw >> 4) - dig_T1)
			* ((temperatura_raw >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;

	t_fine = var1 + var2;
	temperatura = (t_fine * 5 + 128) >> 8;
	temperatura_float = (float) temperatura;

	return (temperatura_float / 100.0f);
}

//salje se naredba da se dobi raw value te se uz pomoc kalibriranih vrijednosti
//dobiva prava vrijednost tlaka
float BMP280_dobij_tlak() {

	int32_t tlak_raw;
	float tlak_float;
	char naredba[3];

	HAL_StatusTypeDef status1 = HAL_ERROR;
	HAL_StatusTypeDef status2 = HAL_ERROR;

	naredba[0] = 0xF7 | 0x80;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	status1 = HAL_SPI_Transmit(&hspi2, (uint8_t*) naredba, 1, 100);
	status2 = HAL_SPI_Receive(&hspi2, (uint8_t*) naredba, 3, 100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	if (status1 != HAL_OK || status2 != HAL_OK) {
		return 0;
	};

	tlak_raw = (naredba[0] << 12) | (naredba[1] << 4) | (naredba[2] >> 4);

	int64_t var1, var2, p;
	uint32_t tlak;

	var1 = (int64_t) t_fine - 128000;
	var2 = var1 * var1 * (int64_t) dig_P6;
	var2 = var2 + ((var1 * (int64_t) dig_P5) << 17);
	var2 = var2 + ((int64_t) dig_P4 << 35);
	var1 = ((var1 * var1 * (int64_t) dig_P3) >> 8)
			+ ((var1 * (int64_t) dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) dig_P1) >> 33;

	if (var1 == 0) {
		return 0;
	}
	p = 1048576 - tlak_raw;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t) dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t) dig_P7) << 4);
	tlak = (uint32_t) p / 256;

	tlak_float = (float) tlak;
	return (tlak_float / 100.0f);

}
//BMP280
/////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
///GPIO INTERRUPTS

// EXTI4 interrupt handler
//tipkalo za promjeniti lcd zaslon
void EXTI4_IRQHandler(void) {
	// Provjeri jel interrupt GPIO_PIN_4
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) {

		//pokrene se timer za debouncing
		HAL_TIM_Base_Start_IT(&htim6);

		// Očisti interrupt flag
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
	}
}

// EXTI9_5 interrupt handler
//tipkalo za ugasiti buzzer i ledice
void EXTI9_5_IRQHandler(void) {
	// Provjeri jel interrupt GPIO_PIN_5
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET) {

		//pokrene se timer za debouncing
		HAL_TIM_Base_Start_IT(&htim7);

		// Očisti interrupt flag
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
	}
}
;
;

////GPIO INTERRUPTS
///////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
/////TIMER INTERRUPTS
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//timer za mjenjanje lcd zaslona
	if (htim->Instance == TIM6) {
		//stopiraj timer
		HAL_TIM_Base_Stop_IT(&htim6);

		//resetiraj timer
		__HAL_TIM_SET_COUNTER(&htim6, 0);

		//počisti interrupt flag
		__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

		//promjeni bool vrijednost da se promjeni zaslon u while petlji
		lcd_zaslon_temperatura_tlak = false;
	}
	//timer za gasenje buzzera i ledica
	else if (htim->Instance == TIM7) {
		//stopiraj timer
		HAL_TIM_Base_Stop_IT(&htim7);

		//resetiraj timer
		__HAL_TIM_SET_COUNTER(&htim7, 0);

		//počisti interrupt flag
		__HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);

		//stavi vrijednost u true pa ce se ugasiti buzzer i ledice u while petlji
		iskljuci_buzzer_ledice = true;
	}
	//timer za treperenje ledica
	else if (htim->Instance == TIM10) {
		//treperi/togglaj pin na kojem su spojene ledice
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	}
}
/////TIMER INTERRUPTS
//////////////////////////////////////////////////////////////////////////////////

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	//tu nadalje inicijalizacija svih potrebnih stvari
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_TIM1_Init();
	MX_TIM10_Init();

	lcd_inicijalizacija();

	BMP280_inicijalizacija();

	float temperatura;

	//uz pomoc ovoga se mjenja duty cycle na pwmu za buzzer
	TIM1->CCR1 = 100;

	while (1) {
		//procitaj temperaturu
		temperatura = BMP280_dobij_temperaturu();

		//ako je veca od 29 i ako je varijabla iskljuci_buzzer_ledice == false
		//znaci da mozemo upaliti timere za buzzer i ledice
		if (temperatura > 29 && !iskljuci_buzzer_ledice) {
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_Start_IT(&htim10);
		}

		//ako je iskljuci_buzzer_ledice == true, moramo iskljuciti timere za buzzer
		//i ledice i staviti pin od ledica u 0
		if (iskljuci_buzzer_ledice) {
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim10);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		}

		//ako je temperatura manja od 29 iskljucujemo timere za buzzer i ledice
		//i stavljamo iskljuci_buzzer_ledice varijablu u false tako da kada
		//temperatura poraste sljedeci put iznad 29 da se mogu buzzer i ledice
		//ponovno upaliti
		if (temperatura < 29) {
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim10);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
			iskljuci_buzzer_ledice = false;
		}

		//ako je true stavljamo temperaturu
		if (lcd_zaslon_temperatura_tlak) {
			lcd_prazan_zaslon();
			lcd_posalji_podatke("  Temperatura");
			lcd_redak_stupac(1, 0);
			lcd_posalji_podatke("    %.2f C", temperatura);
		}
		//ako je false stavlja se tlak
		else {
			lcd_prazan_zaslon();
			lcd_posalji_podatke("      Tlak");
			lcd_redak_stupac(1, 0);
			lcd_posalji_podatke("   %.2f hPa", BMP280_dobij_tlak());
			lcd_zaslon_temperatura_tlak = true;
		}

		//malo delaya u beskonacnoj while petlji
		HAL_Delay(1000);
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 16;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 320;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 64 - 1;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 62500;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 64 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 62500;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 64 - 1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 62500;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
	GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC_0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC_3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB4 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Enable and set up EXTI4 interrupt
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	// Enable and set up EXTI9_5 interrupt
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
