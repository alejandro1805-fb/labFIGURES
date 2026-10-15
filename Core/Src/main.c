/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Carro autónomo STM32F103 (Blue Pill)
  *                   - Puente H TB6612FNG (avance / reversa / freno)
  *                   - 2 encoders HC-020K (medición de distancia y giros)
  *                   - Sensor ultrasónico HC-SR04 (se detiene a <= 10 cm)
  *                   - Recorre un polígono regular N lados, L cm, V vueltas
  *                   - Corrección acumulada de pulsos (inercia y redondeo)
  *                   - Modos de prueba y calibración
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  ADELANTE = 0,
  ATRAS    = 1
} Sentido;

/* Lleva la cuenta de lo que cada rueda DEBIÓ recorrer y lo que REALMENTE
   recorrió. Así, si un tramo se pasa por 1 pulso, el siguiente tramo del
   mismo tipo es 1 pulso más corto y el error no se acumula.             */
typedef struct
{
  float    ideal_A;      /* Pulsos ideales acumulados, rueda izquierda */
  float    ideal_B;      /* Pulsos ideales acumulados, rueda derecha   */
  uint32_t real_A;       /* Pulsos contados de verdad, rueda izquierda */
  uint32_t real_B;       /* Pulsos contados de verdad, rueda derecha   */
  float    exceso_A;     /* Pulsos que la rueda sigue girando por      */
  float    exceso_B;     /* inercia después de frenar (se aprende)     */
} Acumulado;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* =====================================================================
   MODO DE FUNCIONAMIENTO
   0 = RECORRIDO del polígono
   1 = PRUEBA de motores y encoders (uno por uno)
   2 = CALIBRAR GIRO: gira en su sitio NUM_LADOS veces sin avanzar.
       Debe terminar mirando EXACTAMENTE hacia donde empezó.
   3 = CALIBRAR DISTANCIA: avanza en línea recta DIST_CALIBRACION_CM.
   ===================================================================== */

#define MODO                   0

/* =====================================================================
   PARÁMETROS DEL RECORRIDO  (CAMBIAR AQUÍ)
   ===================================================================== */

#define NUM_LADOS              4        /* Número de lados: 3 a 6         */
#define LONGITUD_LADO_CM       30.0f    /* Longitud de cada lado en cm    */
#define NUM_VUELTAS            3        /* Veces que recorre el polígono  */

/* =====================================================================
   CALIBRACIÓN  (ver instrucciones)
   ===================================================================== */

/* Si el giro se queda corto súbelo, si se pasa bájalo.
   Fórmula: FACTOR_nuevo = FACTOR_actual * 360 / grados_que_giró_en_total
   (en MODO 2 con NUM_LADOS = 4).                                         */
#define FACTOR_GIRO            1.00f

/* Fórmula: FACTOR_nuevo = FACTOR_actual * distancia_pedida / distancia_medida
   (en MODO 3).                                                           */
#define FACTOR_DISTANCIA       1.00f

#define DIST_CALIBRACION_CM    100.0f   /* Distancia de prueba en MODO 3  */

/* Pulsos que se estima que la rueda sigue girando tras frenar.
   El programa lo va corrigiendo solo en cada tramo.                      */
#define EXCESO_INICIAL         1.0f

/* =====================================================================
   GEOMETRÍA DEL CARRO
   ===================================================================== */

#define DIAMETRO_RUEDA_CM      6.2f     /* Ruedas motrices (con encoder)  */

/* Distancia entre el CENTRO de la rueda izquierda y el CENTRO de la
   rueda derecha. MEDIRLA EN TU CARRO.                                   */
#define ANCHO_EJE_CM           13.0f

/* Disco del HC-020K: 20 ranuras, se cuentan subida y bajada -> 40 pulsos */
#define RANURAS_ENCODER        20
#define FLANCOS_POR_RANURA     2
#define PULSOS_POR_VUELTA      (RANURAS_ENCODER * FLANCOS_POR_RANURA)

#define PI_F                   3.14159265f
#define CM_POR_PULSO           ((PI_F * DIAMETRO_RUEDA_CM) / (float)PULSOS_POR_VUELTA)

/* =====================================================================
   VELOCIDADES (en % de PWM)
   Más lento = más preciso (menos patinaje y menos inercia al frenar).
   Si una rueda se queda trabada al final de un tramo, sube VEL_MINIMA.
   ===================================================================== */

#define VEL_RECTA              55
#define VEL_GIRO               50
#define VEL_LENTA              38       /* Al final de cada tramo         */
#define VEL_MINIMA             30       /* Ninguna rueda baja de esto     */
#define VEL_PRUEBA             60       /* Velocidad en MODO 1            */
#define PULSOS_DESACELERAR     8        /* Pulsos antes del final         */
#define KP_SINCRONIA           3        /* % de PWM por pulso de diferencia */
#define CORRECCION_MAX         15       /* Límite de la corrección (%)    */

/* Si un motor gira al revés, cambia su valor a 1 (o invierte sus cables) */
#define INVERTIR_MOTOR_A       0        /* Motor A = rueda IZQUIERDA      */
#define INVERTIR_MOTOR_B       0        /* Motor B = rueda DERECHA        */

/* =====================================================================
   SENSOR ULTRASÓNICO
   ===================================================================== */

#define DISTANCIA_LIMITE       10       /* Frena a 10 cm o menos          */
#define DISTANCIA_REANUDAR     12       /* Reanuda por encima de 12 cm    */
#define TIMEOUT_ECHO           30000    /* 30 ms                          */
#define PERIODO_MEDICION_MS    60       /* Entre disparos del HC-SR04     */

/* =====================================================================
   SEGURIDAD
   ===================================================================== */

#define TIMEOUT_ATASCO_MS      1500
#define PULSOS_MINIMOS_PRUEBA  5

/* =====================================================================
   OTROS
   ===================================================================== */

#define PAUSA_ENTRE_TRAMOS_MS  400
#define ESPERA_INICIAL_MS      2000
#define DEBOUNCE_US            1000     /* Filtro de ruido del encoder    */
#define PWM_PERIODO            3600     /* 72 MHz / 3600 = 20 kHz         */

/* =====================================================================
   PINES
   ===================================================================== */

/* HC-SR04 */
#define ECHO_PORT     GPIOA
#define ECHO_PIN      GPIO_PIN_0
#define TRIG_PORT     GPIOA
#define TRIG_PIN      GPIO_PIN_1

/* TB6612FNG */
#define AIN1_PORT     GPIOA
#define AIN1_PIN      GPIO_PIN_2
#define AIN2_PORT     GPIOA
#define AIN2_PIN      GPIO_PIN_3
#define BIN1_PORT     GPIOA
#define BIN1_PIN      GPIO_PIN_4
#define BIN2_PORT     GPIOA
#define BIN2_PIN      GPIO_PIN_5
#define STBY_PORT     GPIOB
#define STBY_PIN      GPIO_PIN_0
/* PWMA -> PA6 (TIM3_CH1)   PWMB -> PA7 (TIM3_CH2) */

/* Encoders HC-020K */
#define ENC_A_PORT    GPIOB
#define ENC_A_PIN     GPIO_PIN_6        /* Rueda izquierda */
#define ENC_B_PORT    GPIOB
#define ENC_B_PIN     GPIO_PIN_7        /* Rueda derecha   */

/* LED de la Blue Pill (activo en bajo) */
#define LED_PORT      GPIOC
#define LED_PIN       GPIO_PIN_13
#define LED_ON()      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()     HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)

/* Validación de parámetros en compilación */
#if (NUM_LADOS < 3) || (NUM_LADOS > 6)
#error "NUM_LADOS debe estar entre 3 y 6"
#endif

#if (NUM_VUELTAS < 1)
#error "NUM_VUELTAS debe ser 1 o más"
#endif

#if (MODO < 0) || (MODO > 3)
#error "MODO debe ser 0, 1, 2 o 3"
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* Variables compartidas con la interrupción de los encoders */
volatile uint32_t cuenta_A = 0;
volatile uint32_t cuenta_B = 0;
volatile uint32_t objetivo_A = 0;
volatile uint32_t objetivo_B = 0;
volatile uint8_t  fin_A = 0;
volatile uint8_t  fin_B = 0;
volatile uint8_t  tramo_activo = 0;
volatile uint32_t ultimo_A_us = 0;
volatile uint32_t ultimo_B_us = 0;

/* En MODO 1 el LED parpadea con cada pulso del encoder probado */
volatile uint8_t  led_encoder = 0;

/* Resultados (visibles en Live Expressions del depurador) */
volatile uint32_t prueba_izq_adelante = 0;
volatile uint32_t prueba_izq_atras    = 0;
volatile uint32_t prueba_der_adelante = 0;
volatile uint32_t prueba_der_atras    = 0;
volatile uint8_t  codigo_error        = 0;

/* Acumuladores separados para rectas y giros */
static Acumulado acc_recta;
static Acumulado acc_giro;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */

static uint32_t Micros(void);
static uint32_t Medir_Distancia_cm(void);

static void MotorA_Direccion(Sentido s);
static void MotorB_Direccion(Sentido s);
static void MotorA_Freno(void);
static void MotorB_Freno(void);
static void MotorA_Libre(void);
static void MotorB_Libre(void);
static void MotorA_Velocidad(int32_t pct);
static void MotorB_Velocidad(int32_t pct);

static void Iniciar_Acumulado(Acumulado *acc);
static uint32_t Calcular_Objetivo(float ideal, uint32_t real, float exceso);
static void Arrancar_Motores(Sentido sA, Sentido sB);
static void Ajustar_Velocidades(int32_t vel, uint32_t objA, uint32_t objB);
static void Pausar_Por_Obstaculo(Sentido sA, Sentido sB);
static void Ejecutar_Tramo(Sentido sA, Sentido sB, float pulsos,
                           Acumulado *acc, int32_t vel,
                           uint8_t vigilar_obstaculo);

static void Senal_Error(uint8_t n);
static void Falla(uint8_t n);

#if (MODO == 1)
static uint32_t Probar_Motor(uint8_t motor, Sentido s, uint32_t ms);
static void Prueba_Hardware(void);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =====================================================================
   TIEMPO EN MICROSEGUNDOS (usa SysTick, no interfiere con TIM2)
   ===================================================================== */

static uint32_t Micros(void)
{
  uint32_t ms  = HAL_GetTick();
  uint32_t val = SysTick->VAL;

  if (ms != HAL_GetTick())
  {
    ms  = HAL_GetTick();
    val = SysTick->VAL;
  }

  return (ms * 1000U) +
         ((SysTick->LOAD + 1U - val) / (SystemCoreClock / 1000000U));
}

/* =====================================================================
   HC-SR04: devuelve la distancia en cm (999 si no hay eco)
   ===================================================================== */

static uint32_t Medir_Distancia_cm(void)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
  while (__HAL_TIM_GET_COUNTER(&htim2) < 10) { }
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
  {
    if (__HAL_TIM_GET_COUNTER(&htim2) > TIMEOUT_ECHO)
    {
      return 999;
    }
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
  {
    if (__HAL_TIM_GET_COUNTER(&htim2) > TIMEOUT_ECHO)
    {
      return 999;
    }
  }

  return __HAL_TIM_GET_COUNTER(&htim2) / 58;
}

/* =====================================================================
   CONTROL DEL TB6612FNG
   ===================================================================== */

static void MotorA_Direccion(Sentido s)
{
#if INVERTIR_MOTOR_A
  s = (s == ADELANTE) ? ATRAS : ADELANTE;
#endif
  if (s == ADELANTE)
  {
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
  }
}

static void MotorB_Direccion(Sentido s)
{
#if INVERTIR_MOTOR_B
  s = (s == ADELANTE) ? ATRAS : ADELANTE;
#endif
  if (s == ADELANTE)
  {
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
  }
}

static void MotorA_Freno(void)
{
  HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
}

static void MotorB_Freno(void)
{
  HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
}

static void MotorA_Libre(void)
{
  HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
}

static void MotorB_Libre(void)
{
  HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
}

static void MotorA_Velocidad(int32_t pct)
{
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(pct * PWM_PERIODO / 100));
}

static void MotorB_Velocidad(int32_t pct)
{
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(pct * PWM_PERIODO / 100));
}

/* =====================================================================
   SEÑALES DE ERROR CON EL LED
   1 parpadeo  = rueda IZQUIERDA
   2 parpadeos = rueda DERECHA
   3 parpadeos = las dos
   ===================================================================== */

static void Senal_Error(uint8_t n)
{
  codigo_error = n;

  while (1)
  {
    for (uint8_t i = 0; i < n; i++)
    {
      LED_ON();
      HAL_Delay(250);
      LED_OFF();
      HAL_Delay(250);
    }
    HAL_Delay(1500);
  }
}

static void Falla(uint8_t n)
{
  tramo_activo = 0;
  MotorA_Velocidad(0);
  MotorB_Velocidad(0);
  MotorA_Freno();
  MotorB_Freno();
  HAL_Delay(300);
  MotorA_Libre();
  MotorB_Libre();
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
  Senal_Error(n);
}

/* =====================================================================
   MODO 1: PRUEBA DE HARDWARE
   ===================================================================== */

#if (MODO == 1)

static uint32_t Probar_Motor(uint8_t motor, Sentido s, uint32_t ms)
{
  uint32_t pulsos;

  __disable_irq();
  cuenta_A = 0;
  cuenta_B = 0;
  tramo_activo = 0;
  __enable_irq();

  led_encoder = motor;

  if (motor == 1)
  {
    MotorA_Velocidad(VEL_PRUEBA);
    MotorA_Direccion(s);
  }
  else
  {
    MotorB_Velocidad(VEL_PRUEBA);
    MotorB_Direccion(s);
  }

  HAL_Delay(ms);

  MotorA_Freno();
  MotorB_Freno();
  HAL_Delay(300);
  MotorA_Libre();
  MotorB_Libre();
  MotorA_Velocidad(0);
  MotorB_Velocidad(0);

  led_encoder = 0;
  LED_OFF();

  pulsos = (motor == 1) ? cuenta_A : cuenta_B;
  return pulsos;
}

static void Prueba_Hardware(void)
{
  uint8_t falla = 0;

  LED_ON();
  HAL_Delay(ESPERA_INICIAL_MS);
  LED_OFF();

  prueba_izq_adelante = Probar_Motor(1, ADELANTE, 1500);
  HAL_Delay(700);
  prueba_izq_atras    = Probar_Motor(1, ATRAS,    1500);
  HAL_Delay(1500);
  prueba_der_adelante = Probar_Motor(2, ADELANTE, 1500);
  HAL_Delay(700);
  prueba_der_atras    = Probar_Motor(2, ATRAS,    1500);

  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);

  if ((prueba_izq_adelante < PULSOS_MINIMOS_PRUEBA) ||
      (prueba_izq_atras    < PULSOS_MINIMOS_PRUEBA))
  {
    falla |= 1;
  }

  if ((prueba_der_adelante < PULSOS_MINIMOS_PRUEBA) ||
      (prueba_der_atras    < PULSOS_MINIMOS_PRUEBA))
  {
    falla |= 2;
  }

  if (falla)
  {
    Senal_Error(falla);
  }

  while (1)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(1000);
  }
}

#endif /* MODO == 1 */

/* =====================================================================
   LÓGICA DE MOVIMIENTO
   ===================================================================== */

static void Iniciar_Acumulado(Acumulado *acc)
{
  acc->ideal_A  = 0.0f;
  acc->ideal_B  = 0.0f;
  acc->real_A   = 0;
  acc->real_B   = 0;
  acc->exceso_A = EXCESO_INICIAL;
  acc->exceso_B = EXCESO_INICIAL;
}

/* Pulsos que hay que pedirle a la rueda en este tramo:
   lo que debería llevar - lo que ya lleva - lo que se pasará por inercia */
static uint32_t Calcular_Objetivo(float ideal, uint32_t real, float exceso)
{
  float falta = ideal - (float)real - exceso;

  if (falta < 1.0f) falta = 1.0f;

  return (uint32_t)(falta + 0.5f);
}

static void Arrancar_Motores(Sentido sA, Sentido sB)
{
  __disable_irq();
  if (!fin_A) MotorA_Direccion(sA);
  if (!fin_B) MotorB_Direccion(sB);
  __enable_irq();
}

/* Mantiene ambas ruedas sincronizadas y desacelera al final */
static void Ajustar_Velocidades(int32_t vel, uint32_t objA, uint32_t objB)
{
  int32_t a = (int32_t)cuenta_A;
  int32_t b = (int32_t)cuenta_B;
  int32_t error = a - b;               /* > 0: la izquierda va adelantada */
  int32_t corr;
  int32_t vA, vB;

  uint32_t restA = (objA > (uint32_t)a) ? (objA - (uint32_t)a) : 0;
  uint32_t restB = (objB > (uint32_t)b) ? (objB - (uint32_t)b) : 0;
  uint32_t restante = (restA < restB) ? restA : restB;

  int32_t base = vel;
  if ((restante <= PULSOS_DESACELERAR) && (VEL_LENTA < vel))
  {
    base = VEL_LENTA;
  }

  corr = KP_SINCRONIA * error;
  if (corr >  CORRECCION_MAX) corr =  CORRECCION_MAX;
  if (corr < -CORRECCION_MAX) corr = -CORRECCION_MAX;

  vA = base - corr;
  vB = base + corr;

  if (vA < VEL_MINIMA) vA = VEL_MINIMA;
  if (vB < VEL_MINIMA) vB = VEL_MINIMA;

  MotorA_Velocidad(vA);
  MotorB_Velocidad(vB);
}

static void Pausar_Por_Obstaculo(Sentido sA, Sentido sB)
{
  uint32_t d;

  MotorA_Freno();
  MotorB_Freno();

  do
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(PERIODO_MEDICION_MS);
    d = Medir_Distancia_cm();
  } while (d <= DISTANCIA_REANUDAR);

  LED_ON();
  Arrancar_Motores(sA, sB);
}

/* Ejecuta un tramo de "pulsos" ideales (puede tener decimales).
   Usa el acumulado para compensar redondeo e inercia de tramos previos. */
static void Ejecutar_Tramo(Sentido sA, Sentido sB, float pulsos,
                           Acumulado *acc, int32_t vel,
                           uint8_t vigilar_obstaculo)
{
  uint32_t t_medicion = 0;
  uint32_t prev_A = 0;
  uint32_t prev_B = 0;
  uint32_t t_mov_A;
  uint32_t t_mov_B;
  uint32_t ahora;
  uint32_t objA;
  uint32_t objB;
  uint32_t final_A;
  uint32_t final_B;
  float    medido;

  /* Objetivo de cada rueda para este tramo */
  acc->ideal_A += pulsos;
  acc->ideal_B += pulsos;
  objA = Calcular_Objetivo(acc->ideal_A, acc->real_A, acc->exceso_A);
  objB = Calcular_Objetivo(acc->ideal_B, acc->real_B, acc->exceso_B);

  __disable_irq();
  cuenta_A = 0;
  cuenta_B = 0;
  objetivo_A = objA;
  objetivo_B = objB;
  fin_A = 0;
  fin_B = 0;
  tramo_activo = 1;
  __enable_irq();

  MotorA_Velocidad(vel);
  MotorB_Velocidad(vel);
  Arrancar_Motores(sA, sB);

  t_mov_A = HAL_GetTick();
  t_mov_B = t_mov_A;

  while (!(fin_A && fin_B))
  {
    ahora = HAL_GetTick();

    if (vigilar_obstaculo &&
        ((ahora - t_medicion) >= PERIODO_MEDICION_MS))
    {
      t_medicion = ahora;

      if (Medir_Distancia_cm() <= DISTANCIA_LIMITE)
      {
        Pausar_Por_Obstaculo(sA, sB);
        ahora      = HAL_GetTick();
        t_medicion = ahora;
        t_mov_A    = ahora;
        t_mov_B    = ahora;
      }
    }

    if (cuenta_A != prev_A) { prev_A = cuenta_A; t_mov_A = ahora; }
    if (cuenta_B != prev_B) { prev_B = cuenta_B; t_mov_B = ahora; }

    if (!fin_A && ((ahora - t_mov_A) > TIMEOUT_ATASCO_MS)) Falla(1);
    if (!fin_B && ((ahora - t_mov_B) > TIMEOUT_ATASCO_MS)) Falla(2);

    Ajustar_Velocidades(vel, objA, objB);
    HAL_Delay(5);
  }

  tramo_activo = 0;

  /* Esperar a que el carro se detenga del todo. Los pulsos que lleguen
     aquí (inercia) también se cuentan.                                  */
  HAL_Delay(PAUSA_ENTRE_TRAMOS_MS);

  __disable_irq();
  final_A = cuenta_A;
  final_B = cuenta_B;
  __enable_irq();

  MotorA_Libre();
  MotorB_Libre();

  /* Actualizar lo realmente recorrido */
  acc->real_A += final_A;
  acc->real_B += final_B;

  /* Aprender cuánto se pasa cada rueda por inercia */
  medido = (float)final_A - (float)objA;
  if (medido < 0.0f) medido = 0.0f;
  acc->exceso_A = 0.5f * acc->exceso_A + 0.5f * medido;

  medido = (float)final_B - (float)objB;
  if (medido < 0.0f) medido = 0.0f;
  acc->exceso_B = 0.5f * acc->exceso_B + 0.5f * medido;
}

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  float pulsos_lado;
  float pulsos_giro;
  float angulo_exterior;
  float arco_giro_cm;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  MotorA_Libre();
  MotorB_Libre();
  MotorA_Velocidad(0);
  MotorB_Velocidad(0);

  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);

#if (MODO == 1)
  Prueba_Hardware();      /* No regresa */
#endif

  Iniciar_Acumulado(&acc_recta);
  Iniciar_Acumulado(&acc_giro);

  /* -------------------------------------------------------------
     CÁLCULOS DEL POLÍGONO (con decimales; el acumulado se encarga
     del redondeo para que el error no crezca vuelta tras vuelta)
     ------------------------------------------------------------- */

  pulsos_lado = (LONGITUD_LADO_CM * FACTOR_DISTANCIA) / CM_POR_PULSO;

  angulo_exterior = 360.0f / (float)NUM_LADOS;
  arco_giro_cm    = (angulo_exterior / 360.0f) * PI_F * ANCHO_EJE_CM * FACTOR_GIRO;
  pulsos_giro     = arco_giro_cm / CM_POR_PULSO;

  LED_ON();
  HAL_Delay(ESPERA_INICIAL_MS);

#if (MODO == 0)

  /* ----------------------- RECORRIDO ------------------------- */
  for (uint32_t vuelta = 0; vuelta < NUM_VUELTAS; vuelta++)
  {
    for (uint32_t lado = 0; lado < NUM_LADOS; lado++)
    {
      Ejecutar_Tramo(ADELANTE, ADELANTE, pulsos_lado, &acc_recta, VEL_RECTA, 1);
      Ejecutar_Tramo(ATRAS,    ADELANTE, pulsos_giro, &acc_giro,  VEL_GIRO,  0);
    }
  }

#elif (MODO == 2)

  /* ------------------- CALIBRAR GIRO ------------------------- */
  for (uint32_t i = 0; i < NUM_LADOS; i++)
  {
    Ejecutar_Tramo(ATRAS, ADELANTE, pulsos_giro, &acc_giro, VEL_GIRO, 0);
  }

#elif (MODO == 3)

  /* ----------------- CALIBRAR DISTANCIA ---------------------- */
  Ejecutar_Tramo(ADELANTE, ADELANTE,
                 (DIST_CALIBRACION_CM * FACTOR_DISTANCIA) / CM_POR_PULSO,
                 &acc_recta, VEL_RECTA, 1);

#endif

  MotorA_Libre();
  MotorB_Libre();
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* LED parpadeando: terminado */
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration (HSE 8 MHz x 9 = 72 MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief TIM2: contador de 1 us (72 MHz / 72)
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief TIM3: PWM 20 kHz en CH1 (PA6 -> PWMA) y CH2 (PA7 -> PWMB)
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_PERIODO - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, TRIG_PIN | AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  /* PA0 -> ECHO */
  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

  /* PA1 TRIGGER, PA2 AIN1, PA3 AIN2, PA4 BIN1, PA5 BIN2 */
  GPIO_InitStruct.Pin = TRIG_PIN | AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB0 -> STBY */
  GPIO_InitStruct.Pin = STBY_PIN;
  HAL_GPIO_Init(STBY_PORT, &GPIO_InitStruct);

  /* PC13 -> LED */
  GPIO_InitStruct.Pin = LED_PIN;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  /* PA6 -> PWMA, PA7 -> PWMB */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB6 -> Encoder izquierdo, PB7 -> Encoder derecho */
  GPIO_InitStruct.Pin = ENC_A_PIN | ENC_B_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}


/* USER CODE BEGIN 4 */

/* =====================================================================
   INTERRUPCIÓN DE LOS ENCODERS (PB6 y PB7 comparten EXTI9_5)
   Si CubeMX genera este handler en stm32f1xx_it.c, BORRA este de aquí.
   ===================================================================== */

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ENC_A_PIN);
  HAL_GPIO_EXTI_IRQHandler(ENC_B_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t ahora = Micros();

  if (GPIO_Pin == ENC_A_PIN)
  {
    if ((ahora - ultimo_A_us) < DEBOUNCE_US) return;
    ultimo_A_us = ahora;

    cuenta_A++;
    if (led_encoder == 1) HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

    if (tramo_activo && !fin_A && (cuenta_A >= objetivo_A))
    {
      MotorA_Freno();
      fin_A = 1;
    }
  }
  else if (GPIO_Pin == ENC_B_PIN)
  {
    if ((ahora - ultimo_B_us) < DEBOUNCE_US) return;
    ultimo_B_us = ahora;

    cuenta_B++;
    if (led_encoder == 2) HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

    if (tramo_activo && !fin_B && (cuenta_B >= objetivo_B))
    {
      MotorB_Freno();
      fin_B = 1;
    }
  }
}

/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
