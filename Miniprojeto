/*Diogo Luís Marques Gaspar - 2202128
Tomás da Silva Brites - 2202121
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEAU- Licenciatura em Engenharia Automóvel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos


O seguinte trabalho consiste na simulação da implementação de um indicador de mudanças numa motorizada.
Neste projeto sera controlado a posição do pedal que coloca as mudanças através de dois sensores de hall
e a temperatura do motor através de um sensor digital. Quando a motorizada se encontra em ponto morto
será aceso um led verde. Quando a temperatura de motor ultrapassar os 100 graus Celsius será acesso
um led vermelho que variará a sua intensidade. Através de um display será ilustrado a mudança em que a
mota se encontra, a temperatura do motor, e a tensão de saída de um dos sensores de hall, convertida
 através de ADC.

*/

#include <math.h>
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"


//Bibliotecas para o TFT
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Wire.h>

//Biblioteca para o sensor de temperatura
#include <Adafruit_Sensor.h>
#include <DHT.h>


/* Definicoes de pinos */
#define led_neutral 4
#define led_PWM 5
#define button 0
#define hall_up 25
#define hall_down 26

//Sensor Temperatura
#define DHTPIN 27        			// Pino digital do sensor de temperatura
#define DHTTYPE    DHT11 			// Defenição do tipo de sensor DHT
DHT dht(DHTPIN, DHTTYPE);


//Pinos do TFT
#define TFT_CS   2
#define TFT_DC   15
#define DISTC 12	// Distância entre caracteres em pixels
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

//Define do ADC
#define hall_analog 34
#define ADC_RESOLUTION 12
#define VREF_PLUS  3.3
#define VREF_MINUS  0.0

/* Definicoes de prototipos das tarefas */
void vTaskshift_neutral (void * pvParameters);
void vTaskmove_up (void * pvParameters);
void vTaskmove_down (void * pvParameters);
void vTaskdisplay( void *pvParameters );
void vTaskhallup_volt( void *pvParameters );
void vTasktemp( void *pvParameters );

/*Definicoes dos prototipos das interrupcoes */
void IRAM_ATTR neutral();
void IRAM_ATTR pedal_up();
void IRAM_ATTR pedal_down();



/*Queues*/

QueueHandle_t shiftQueue;
QueueHandle_t hallvoltQueue;
QueueHandle_t tempQueue;



//Semaforos

//Semaforo para sincronizacao tarefas do botão com interrupcao dos mesmos
SemaphoreHandle_t neutralSemaphore;

//Semaforo para sincronizacao tarefas dos sensores de hall com interrupcao dos mesmos
SemaphoreHandle_t pedal_upSemaphore;
SemaphoreHandle_t pedal_downSemaphore;

//Semaforo Mutex para proteção do display
SemaphoreHandle_t mutexSemaphore;

void setup()
{


		vSemaphoreCreateBinary(neutralSemaphore);
		vSemaphoreCreateBinary(pedal_upSemaphore);
		vSemaphoreCreateBinary(pedal_downSemaphore);

		mutexSemaphore = xSemaphoreCreateMutex();

	  //Configuração dos pinos de entrada
	  pinMode(button, INPUT_PULLUP);		  // Configuracao do pino 0 como entrada com pull up
	  pinMode (hall_up, INPUT);
	  pinMode (hall_analog, INPUT);
	  pinMode (hall_down, INPUT);
	  pinMode(DHTPIN, INPUT);

	  //Configuração dos pinos de saída
	  pinMode(led_neutral, OUTPUT);
	  pinMode(led_PWM, OUTPUT);


	  // Interrupção para o botão
	  attachInterrupt(digitalPinToInterrupt(button), neutral, FALLING);


	  // Interrupções para os sensores de hall
	  attachInterrupt(digitalPinToInterrupt(hall_up), pedal_up, RISING);
	  attachInterrupt(digitalPinToInterrupt(hall_down), pedal_down, RISING);
	  interrupts(); // Permite interrupções


	  Serial.begin(115200); //Inicia a porta serie


	  shiftQueue = xQueueCreate(1, sizeof( int ));		//Queue para armazenar o valor da mudança
	  hallvoltQueue = xQueueCreate(3, sizeof(float));	//Queue para armazenar o valor da tesão do sensor de hall da tarefa move up
	  tempQueue = xQueueCreate(3, sizeof(float));		//Queue para armazenar o valor da temperatura


	  vTaskPrioritySet(NULL, configMAX_PRIORITIES-1);	//Define o setup como prioridade maxima

	  xTaskCreatePinnedToCore(vTaskshift_neutral, "shift neutral", 800, NULL, 2, NULL, 1);

	  xTaskCreatePinnedToCore(vTaskmove_up, "move up", 1024, NULL, 1, NULL, 1);

	  xTaskCreatePinnedToCore(vTaskmove_down, "move down", 1024, NULL, 1, NULL, 1);

	  xTaskCreatePinnedToCore(vTaskdisplay, "display", 4096, NULL, 1,NULL, 1);

	  xTaskCreatePinnedToCore( vTaskhallup_volt, "ADC", 1024, NULL, 1, NULL, 1);

	  xTaskCreatePinnedToCore( vTasktemp, "temp", 1024, NULL, 3, NULL, 1);


}

//Tarefa para leitura da temperatura
void vTasktemp( void *pvParameters )
{
	float temp = 0;

	int freq = 5000;
	int ledChannel = 5;
	int resolution = 8;

	int led_value;
	float sin_value;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	ledcSetup(ledChannel, freq, resolution);
	ledcAttachPin(led_PWM, ledChannel);

	dht.begin();

	for(;;)
	{

	  temp = dht.readTemperature();     //Lê a temperatura do sensor

	  //Erro na leitura da temperatura
	  if (isnan(temp)) {
	  	Serial.println("Failed to read from DHT sensor!");

	  }
	  else{


	  	Serial.println("temp lida");		//Imprime o valor da temperatura lido
	  	Serial.println(temp);
	  }

	 //Envia o valor da temperatura para a queue
     xQueueSendToBack(tempQueue, &temp, 0);


     if(temp >= 100)
     {
    	 for (int x=0; x<180; x++)	//valores entre 0 e 179 pois a função seno é positiva apenas entre 0 e 180 graus
    	   {
    	     // converte graus para radianos e então obtém o valor do seno
    	     sin_value = (sin(x*(3.1412/180)));
    	     led_value = int(sin_value*255);	//o valor do brilho variará entre 0 e 255
    	     ledcWrite(ledChannel, led_value);	//Acende o led
    	     delay(25);
    	   }
     }else
     {
    	 //Apaga o Led
    	 ledcWrite(ledChannel, 0);

     }

    vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
	}

}

/*Tarefa para ler a tensão de saída do sensor de hall associado a tarefa move up através da utilização do ADC */
void vTaskhallup_volt( void *pvParameters )
{
	int analog_value = 0;
	float analog_voltage = 0;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	analogReadResolution(ADC_RESOLUTION);

  for(;;)
  {

	// Acquisição do sinal analógico
    analog_value = analogRead(hall_analog);

    //Conversão do sinal analógico para tensão
    analog_voltage = analog_value * (VREF_PLUS-VREF_MINUS)/(pow (2.0, (float)ADC_RESOLUTION))+VREF_MINUS;

    Serial.println("Tensao do sensor de hall: ");		//Imprime o valor da tensão obtida
    Serial.println(analog_voltage);

    xQueueSendToBack(hallvoltQueue, &analog_voltage, 0);
    vTaskDelayUntil(&xLastWakeTime, (250 / portTICK_PERIOD_MS));
  }
}

//Tarefa para gestão do display
void vTaskdisplay( void *pvParameters )
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int shift_antiga;
	int shift_atual=5;
	float hall_value;

	float temp = 0;
	float temp_antiga = 0;


	dht.begin();

	// inicializar o tft
	tft.begin();
	// colocar fundo a preto
	tft.fillScreen(ILI9341_BLACK);
	// definir orientacao
	tft.setRotation(3);


	for(;;)
	{

		//A tarefa fica bloqueada enquanto o semáforo não for obtido com sucesso
		xSemaphoreTake(mutexSemaphore, portMAX_DELAY);



    	//Desenhar retangulo inicial
    	tft.drawRect(0, 0, 380, 4, ILI9341_WHITE);
    	tft.fillRect(0, 0, 380, 4, ILI9341_WHITE);


		//Temperatura---------------------------------------------------------------------------

    	//recebe o valor da temperatura
    	xQueueReceive(tempQueue, &temp, 0);

    	tft.setTextSize(2.8);						//Selecionar tamanho do texto
		tft.setCursor(55,15);						//Selecionar onde imprime o texto no display
		tft.setTextColor(ILI9341_WHITE);
		tft.println("Temperatura");

		Serial.println("temp recebida");
		Serial.println(temp);

		if(temp != temp_antiga)
		{
			tft.fillRect(50, 50, 150, 30, ILI9341_BLACK);
			tft.setTextSize(3);
			tft.setCursor(50,50);
			tft.print(temp);		//Mostrar Temperatura no display
			tft.print(" ");			//Espaçamento entre o numero e a unidade
			tft.print((char) 167); // Símbolo do grau (extended ASCII)
			tft.print("C");

			temp_antiga = temp;
		}


		//Desenhar retangulo do meio
    	tft.drawRect(0, 85, 380, 4, ILI9341_WHITE);
    	tft.fillRect(0, 85, 380, 4, ILI9341_WHITE);


    	//Desenhar retângulo de fundo para a mudança
    	tft.fillRoundRect(30, 105, 180, 28, 10, ILI9341_GREEN);
    	//Escreve SHIFT
    	tft.setTextSize(3);
    	tft.setCursor(75, 108);
    	tft.println("SHIFT");


    	//Mudança-------------------------------------------------------------------------
		tft.setTextSize(13);
		tft.setTextColor(ILI9341_WHITE);


		//recebe o valor da mudança
    	xQueuePeek(shiftQueue, &shift_atual, 0);



    	//Atualiza o valor da mudança caso seja diferente da anterior
    	if(shift_atual!=shift_antiga){
    				switch(shift_atual){
    					case 1:
    						tft.fillRect(60, 160, 200, 200, ILI9341_BLACK);
    						tft.setCursor(90, 160);
    						tft.print("1");
    						break;
    					case 2:
    						tft.fillRect(60, 160, 200, 200, ILI9341_BLACK);
    						tft.setCursor(90, 160);
    						tft.print("2");
    						break;
    					case 3:
    						tft.fillRect(60, 160, 200, 200, ILI9341_BLACK);
    						tft.setCursor(90, 160);
    						tft.print("3");
    						break;
    					case 4:
    						tft.fillRect(60, 160, 200, 200, ILI9341_BLACK);
    						tft.setCursor(90, 160);
    						tft.print("4");
    						break;
    					default:
    						tft.fillRect(60, 160, 200, 200, ILI9341_BLACK);
    						tft.setCursor(90, 160);
    						tft.print("N");
    						break;
    				}

    				shift_antiga = shift_atual;	//atualiza o valor da mudança na variavel shift_antiga
    	}

    	//Tensão do sensor de hall-------------------------------------------------------------------

		//Desenhar retangulo do meio
    	tft.drawRect(0, 280, 380, 4, ILI9341_WHITE);
    	tft.fillRect(0, 280, 380, 4, ILI9341_WHITE);

    	xQueuePeek(hallvoltQueue, &hall_value, 0);


    	tft.setTextSize(2);
    	tft.setCursor(10,295);
    	tft.setTextColor(ILI9341_WHITE);
    	tft.print("Hall Up: ");
    	tft.print(hall_value);
    	tft.print(" V");


    //Da o mutex
    xSemaphoreGive(mutexSemaphore);
	vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
	}

}


/*Quando corre a interrupção pela atuação do botão esta tarefa irá atualizar o valor da mudança
presente na queue shiftQueue */

void vTaskshift_neutral (void * pvParameters)
{
	int shift;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();


	//pega o semaforo após este ser criado
	xSemaphoreTake(neutralSemaphore,0);

	for(;;)
	{
				//A tarefa fica bloqueada enquanto o semáforo não for obtido com sucesso
				xSemaphoreTake(neutralSemaphore, portMAX_DELAY);

				//recebe o valor da mudança atual presente na queue
				xQueueReceive(shiftQueue, &shift, 0);

				shift = 0;

				//Envia o valor da mudança atual presente na queue
				xQueueSendToFront(shiftQueue, &shift, 0);

				//Imprime o valor da mudança
				Serial.print("Mudanca: ");
				Serial.println(shift);

				//Acende o LED verde
				digitalWrite(led_neutral, HIGH);



	//Aguarda a tarfefa completar 200ms
	vTaskDelayUntil(&xLastWakeTime, (200 / portTICK_PERIOD_MS));
	}

}

/*Quando corre a interrupção pela deteção do sensor de hall quando o pedal se move para cima esta tarefa irá atualizar o valor da mudança
presente na queue shiftQueue */

void vTaskmove_up (void * pvParameters)
{
	int shift;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();


	//pega o semaforo após este ser criado
	xSemaphoreTake(pedal_upSemaphore,0);

	for(;;)
	{

				//A tarefa fica bloqueada enquanto o semáforo não for obtido com sucesso
				xSemaphoreTake(pedal_upSemaphore, portMAX_DELAY);

				//recebe o valor da mudança atual presente na queue
				xQueueReceive(shiftQueue, &shift, 0);

				switch (shift) {
				  case 1:
					  shift = 2;
					  xQueueSendToFront(shiftQueue, &shift, 0);

				  break;
				  case 2:
					  shift = 3;
					  xQueueSendToFront(shiftQueue, &shift, 0);

				  break;
				  case 3:
					  shift = 4;
					  xQueueSendToFront(shiftQueue, &shift, 0);

				  break;
				  default:
					  Serial.print("A mudanca nao esta entre a 1 e a 3 \r\n");
					  xQueueSendToFront(shiftQueue, &shift, 0);

				  break;
				  }

				Serial.print("Mudanca: ");
				Serial.println(shift);

	vTaskDelayUntil(&xLastWakeTime, (250 / portTICK_PERIOD_MS));
	}
}


/*Quando corre a interrupção pela deteção do sensor de hall quando o pedal se move para baixo esta tarefa irá atualizar o valor da mudança
presente na queue shiftQueue */

void vTaskmove_down (void * pvParameters)
{
	int shift;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();


	//pega o semaforo após este ser criado
	xSemaphoreTake(pedal_downSemaphore,0);

	for(;;)
	{
				//A tarefa fica bloqueada enquanto o semáforo não for obtido com sucesso
				xSemaphoreTake(pedal_downSemaphore, portMAX_DELAY);

				//recebe o valor da mudança atual presente na queue
				xQueueReceive(shiftQueue, &shift, 0);

				if(shift == 0){

					shift = 1;

					//envia o valor da mudança atual presente na queue
					xQueueSendToFront(shiftQueue, &shift, 0);

					digitalWrite(led_neutral, LOW);

				} else {

				switch (shift) {
				  case 2:
					  shift = 1;
					  //envia o valor da mudança atual presente na queue
					  xQueueSendToFront(shiftQueue, &shift, 0);

				    break;
				  case 3:
					  shift = 2;
					  //envia o valor da mudança atual presente na queue
					  xQueueSendToFront(shiftQueue, &shift, 0);

				    break;
				  case 4:
					  shift = 3;
					  //envia o valor da mudança atual presente na queue
					  xQueueSendToFront(shiftQueue, &shift, 0);

				    break;
				  default:
					  Serial.print("A mudanca nao esta entre a 2 e a 4 \r\n");
					  //devolve o valor da mudança atual presente na queue
					  xQueueSendToFront(shiftQueue, &shift, 0);

				    break;
				}
				}

				//imprime o valor da mudança
				Serial.print("Mudanca: ");
				Serial.println(shift);

	vTaskDelayUntil(&xLastWakeTime, (250 / portTICK_PERIOD_MS));
	}
}

void  neutral()
{
	//permite ativar um semaforo para ativar a vshift_neutral
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(neutralSemaphore, (signed portBASE_TYPE *)&xHigherPriorityTaskWoken);

}

void  pedal_up()
{
	//permite ativar um semaforo para ativar a vmove_up
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(pedal_upSemaphore, (signed portBASE_TYPE *)&xHigherPriorityTaskWoken);

}

void  pedal_down()
{
	//permite ativar um semaforo para ativar a vmove_down
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(pedal_downSemaphore, (signed portBASE_TYPE *)&xHigherPriorityTaskWoken);

}

void loop()
{
    vTaskDelete(NULL);
}
