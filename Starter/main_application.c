/* Standard includes. */
#include <stdio.h>
#include <conio.h>
#include <string.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* Hardware simulator utility functions */
#include "HW_access.h"

/* SERIAL SIMULATOR CHANNEL TO USE */
#define COM_CH (0)
#define COM_CH1 (1)
#define COM_CH2 (2) 

	/* TASK PRIORITIES */
#define OBRADA_TASK_PRI ( tskIDLE_PRIORITY + (UBaseType_t)1 )
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + (UBaseType_t)3 )
#define TASK_SERIAl_REC_PRI			( tskIDLE_PRIORITY + (UBaseType_t)4 )
#define	SERVICE_TASK_PRI		( tskIDLE_PRIORITY + (UBaseType_t)2 )       


/* TASKS: FORWARD DECLARATIONS */
static void led_bar_tsk(const void* pvParameters); //sluzi za ocitavanje vrednosti prekidaca i manipulisanje start_local promenljivom
static void Primio_kanal_0(const void* pvParameters); //prijem sa senzora 1
static void Primio_kanal_1(const void* pvParameters); //prijem sa senzora 2 
static void LED_bar_Task1(const void* pvParameters);//generise blinkanje dioda na trecem stupcu
static void LED_bar_Task2(const void* pvParameters);//generise blinkanje dioda na cetvrtom stupcu
static void SerialSend_Task(const void* pvParameters); //ispisuje na serijsku 
static void SerialReceive_Task(void* pvParameters); //prijem komandi sa serijske
static void Seg7_ispis_task(void* pvParameters); //ispisivanje trazenih informacija na 7-segmentni displej
static void Serijska_stanje_task(void* pvParameters); //ispis stanja sistema na serijsku svakih 5s
void main_demo(void);

/* TIMER FUNCTIONS*/
static void ispis_tajmer_callback(TimerHandle_t ispis_podaci_tajmer);
static void TimerCallback(TimerHandle_t per_TimerHandle);

/* Globalne promjenljive za generalnu upotrebu */
#define R_BUF_SIZE (32)

const char trigger[] = "XYZ"; //sluzi kao trigger poruka za odgovore sa kanala 0 i kanala 1
unsigned volatile t_point2; //sluzi kao brojac pozicije trigger niza XYZ


/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

/* GLOBAL OS-HANDLES */
static SemaphoreHandle_t LED_INT_BinarySemaphore;
static SemaphoreHandle_t TBE_BS_0, TBE_BS_1, TBE_BS_2;
static SemaphoreHandle_t RXC_BS_0, RXC_BS_1, RXC_BS_2; 
static SemaphoreHandle_t seg7_ispis;
static SemaphoreHandle_t serijska_stanje; 

static TimerHandle_t per_TimerHandle1;
static TimerHandle_t ispis_podaci_tajmer1;


static QueueHandle_t seg7_auto_queue;
static QueueHandle_t serijska_ispis_queue;
static QueueHandle_t serijska_ispis_duzina;
static QueueHandle_t queue_senzor1;
static QueueHandle_t queue_senzor2;
static QueueHandle_t queue_kalibracija1;
static QueueHandle_t queue_kalibracija2;
static QueueHandle_t queue_kalibracija3;
static QueueHandle_t queue_kalibracija4;
static QueueHandle_t queue_kalibracija1_max;
static QueueHandle_t queue_kalibracija1_min;
static QueueHandle_t queue_kalibracija2_max;
static QueueHandle_t queue_kalibracija2_min;
static QueueHandle_t serijska_prijem_niz;
static QueueHandle_t serijska_prijem_duzina;
static QueueHandle_t stanje_sistema;


static void led_bar_tsk(const void* pvParameters) //sluzi za ocitavanje vrednosti prekidaca tj. LED0 na nultom stupcu i manipulisanje start_local promenljivom
{

	uint8_t d;
	uint8_t start_local = 0;
	for (;;)
	{
		if (xSemaphoreTake(LED_INT_BinarySemaphore, portMAX_DELAY) != pdTRUE) { 
			//printf("Error 001\n");
		}

		if (xQueueReceive(stanje_sistema, &start_local, pdMS_TO_TICKS(20)) != pdTRUE) {
			//printf("Error 002\n");
		}


		if (get_LED_BAR(0, &d) != 0) {
			//printf("Error 003\n");
		} //procitaj stanje prvog stubca led bara

		if ((d & (uint8_t)0x01) != (uint8_t)0) { //provjeri da li je pritisnut prvi prekidac na led baru i ako jeste postavi start_local na 1 u suprotnom start_local na 0
			if (set_LED_BAR(1, 0x01) != 0) {
				//printf("Error 004\n");
			}
			start_local = 1;
		}
		else {
			if (set_LED_BAR(1, 0x00) != 0) {
				//printf("Error 005\n");
			}
			start_local = 0;
		}

		if (xQueueSend(stanje_sistema, &start_local, 0U) != pdTRUE) {
			//printf("Error 006\n");
		}

		if (xQueueSend(stanje_sistema, &start_local, 0U) != pdTRUE) {
			//printf("Error 007\n");
		}

		if (xQueueSend(seg7_auto_queue, &start_local, 0U) != pdTRUE) {
			//printf("Error 008\n");
		}


	}
}
static void SerialSend_Task(const void* pvParameters)
{ 
	uint8_t t_point = 0;
	uint8_t r[60];
	uint8_t duzina_niza_ispis = 0;

	for (;;)
	{
		if (xSemaphoreTake(TBE_BS_2, portMAX_DELAY) != pdTRUE) {   
			//printf("Error 009\n");
		}
		// sacekaj da TX registar bude prazan
 
		if (xQueueReceive(serijska_ispis_queue, &r, pdMS_TO_TICKS(20)) != pdTRUE) {
			//printf("Error 010\n");
		}

		//pogledaj da li ima novih vrijednosti ( ceka se 20ms )
		if (xQueueReceive(serijska_ispis_duzina, &duzina_niza_ispis, pdMS_TO_TICKS(20)) != pdTRUE) {
			//printf("Error 011\n");
		}

		//pogledaj da li ima novih vrijednosti ( ceka se 20ms )

		if (t_point < duzina_niza_ispis) { //dok nije ispisan posljednji karakter salji slovo po slovo na kanal 2 
			if (send_serial_character(COM_CH2, r[t_point++]) != 0) {
				printf("Slanje karaktera neispravno\n");
			}
			

		}
		else { //kada se ispise posljednji karakter na kanalu 2 vrati brojacku promenljivu na 0
			t_point = 0;
			duzina_niza_ispis = 0;
		}

		if (t_point2 > (sizeof(trigger) - 1)) //ako je t_point2 = 3 onda je ispisano XYZ i vrati t_point na nulu
			t_point2 = 0;
		send_serial_character(COM_CH, trigger[t_point2]); //ispis XYZ na kanal 0
		send_serial_character(COM_CH1, trigger[t_point2++]); //ispis XYZ na kanal 0
		//xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);// kada se koristi predajni interapt
		vTaskDelay(pdMS_TO_TICKS(200));// kada se koristi vremenski delay
	}
}


static void LED_bar_Task1(const void* pvParameters) { // LED_bar_Task1 se koristi za blinkanje dioda u zavisnosti od vrednosti kalibracije 
	double kalibracija3_local = 0;// kalibracija1 je jednaka kalibraciji3 ali je trebala zbog sinhronizacije taskova pa smo napravili i 2 razlicita reda
	uint8_t i_local = 0; // pomocna promenljiva pomocu koje ogranicava slucaj kada nijedan uslov nije ispunjen tj. ZONA DETEKCIJE                                        
						 // da nam se na pocetku to ispise samo jednom na terminalu, a ne nonstop da se ispisuje (ne radi lepo msm da je zbog atofa)
	for (;;) {

		if (xQueueReceive(queue_kalibracija3, &kalibracija3_local, pdMS_TO_TICKS(20)) != 0) { // smestanje kalibrasane vrednosti sa senzora 1 
			printf("Kalibracija senzora LEVI\n");
		}

		if (kalibracija3_local > (double)50 && kalibracija3_local <= (double)100) {
			printf("LEVI SENZOR - UDALJENA DETEKCIJA\n");     // Generisemo signal frekvencije 100Hz, pola periode svetli, pola ne svetli
			if (set_LED_BAR(2, 0xF0) != 0) {               
				//printf("Error 012\n");					       
			}											   
			vTaskDelay(pdMS_TO_TICKS(5));             
			if (set_LED_BAR(2, 0x00) != 0) {              
				//printf("Error 013\n");
			}
			vTaskDelay(pdMS_TO_TICKS(5));
			i_local = 0;
		}

		else if (kalibracija3_local > (double)0 && kalibracija3_local <= (double)50) {
			printf("LEVI SENZOR - BLISKA DETEKCIJA\n");     // Generisemo signal frekvencije 200Hz, pola periode svetli, pola ne svetli
			if (set_LED_BAR(2, 0xF0) != 0) {               
				//printf("Error 014\n");			               
			}                                             
			vTaskDelay(pdMS_TO_TICKS(2.5));                
			if (set_LED_BAR(2, 0x00) != 0) {                          
				//printf("Error 015\n");
			}
			vTaskDelay(pdMS_TO_TICKS(2.5));
			i_local = 0;
		}

		else if (kalibracija3_local < (double)0) {
			printf("LEVI SENZOR - KONTAKT DETEKCIJA\n");    // Generisemo signal frekvencije 200Hz, celu periodu svetli
			if (set_LED_BAR(2, 0xF0) != 0) {               
				//printf("Error 016\n");                        
			}
			vTaskDelay(pdMS_TO_TICKS(5));                 
			if (set_LED_BAR(2, 0x00) != 0) {
				//printf("Error 017\n");
			}
			i_local = 0;
		}
		else if (i_local == (uint8_t)0) {                  //NEMA DETEKCIJE, NE GENERISEMO NIKAKV SIGNAL
			printf("LEVI SENZOR - NEMA DETEKCIJE\n");
			i_local = 1;
		}
		

	}

}

static void LED_bar_Task2(const void* pvParameters) { // LED_bar_Task2 se koristi za blinkanje dioda u zavisnosti od vrednosti kalibracije 
	double kalibracija4_local = 0;   // kalibracija2 je jednaka kalibraciji4 ali je trebala zbog sinhronizacije taskova pa smo napravili i 2 razlicita reda
	uint8_t i_local = 0;  // pomocna promenljiva pomocu koje ogranicava slucaj kada nijedan uslov nije ispunjen tj. ZONA DETEKCIJE                                        
						 // da nam se na pocetku to ispise samo jednom na terminalu, a ne nonstop da se ispisuje (ne radi lepo msm da je zbog atofa)

	for (;;) {
		if (xQueueReceive(queue_kalibracija4, &kalibracija4_local, pdMS_TO_TICKS(20)) != 0) {
			printf("Kalibracija senzora DESNI\n");
		}
		
		if (kalibracija4_local > (double)50 && kalibracija4_local <= (double)100) {
			printf("DESNI SENZOR - UDALJENA DETEKCIJA\n");      // Generisemo signal frekvencije 100Hz, pola periode svetli, pola ne svetli
			if (set_LED_BAR(3, 0xF0) != 0) {                              
				//printf("Error 018\n");                          
			}                                                
			vTaskDelay(pdMS_TO_TICKS(5));               
			if (set_LED_BAR(3, 0x00) != 0) {              
				//printf("Error 019\n");
			}
			vTaskDelay(pdMS_TO_TICKS(5));
			i_local = 0;
		}

		else if (kalibracija4_local > (double)0 && kalibracija4_local <= (double)50) {
			printf("DESNI SENZOR - BLISKA DETEKCIJA\n");	   // Generisemo signal frekvencije 200Hz, pola periode svetli, pola ne svetli
			if (set_LED_BAR(3, 0xF0) != 0) {               
				//printf("Error 020\n");                        
			}                                              
			vTaskDelay(pdMS_TO_TICKS(2.5));                
			if (set_LED_BAR(3, 0x00) != 0) {                          
				//printf("Error 021\n");
			}
			vTaskDelay(pdMS_TO_TICKS(2.5));
			i_local = 0;
		}

		else if (kalibracija4_local < (double)0) {                       // Generisemo signal frekvencije 200Hz, svetli celu periodu
			printf("DESNI SENZOR - KONTAKT DETEKCIJA\n");
			if (set_LED_BAR(3, 0xF0) != 0) {                          
				//printf("Error 022\n");
			}
			vTaskDelay(pdMS_TO_TICKS(5));                
			if (set_LED_BAR(3, 0x00) != 0) {
				//printf("Error 023\n");
			}
			i_local = 0;
		}
		else if (i_local == (uint8_t)0) {
			printf("DESNI SENZOR - NEMA DETEKCIJE\n");
			i_local = 1;
		}
		
		
	}
}

static void Primio_kanal_0(const void* pvParameters) //prijem sa kanala 0 tj. senzor 1
{

	double senzor1 = 0;   // promeljiva u koju smestamo vrednosti primljene sa kanala 0
	uint8_t cc = 0;   // prvo se primljeno sa kanala nula smesta u ovu promenljivu (2 bajta po 2 bajta)
	uint8_t br_karaktera = 0; //sluzi nam da se pomeramo kroz niz rastojanje_kanal0[6], ako je stigao karakter za kraj poruke, resetuje se na 0
	double kalibracija1_local = 0;  // promenljiva u koju smestamo kalibrisanu vrednost primljenu sa kanala 0
	uint8_t rastojanje_kanal0[6] = { 0 }; // rastajoanje sa senzora 1
	double min = 20, max = 100; // minimalne i maksimalne vrednosti, potrebne za kalibraciju
	const char rastojanje_kanal0_radi[6] = { "64"}; // POTREBNO DA BI POKAZALO DA ZEZA ATOF I DA BI SE POKAZALO DA RADI KOD

	for (;;) {
		if (xSemaphoreTake(RXC_BS_0, portMAX_DELAY) != pdTRUE) { // uzima semafor
			//printf("Error 024\n");
		}

		if (get_serial_character(COM_CH, &cc) != 0) {   // smesta karaktere primljene sa kanala 0 u promenljivu cc
			//printf("Error 025\n");
		}

		if (xQueueReceive(queue_kalibracija1_min, &min, pdMS_TO_TICKS(20)) != pdTRUE) { // ceka vrednosti sa senzora 1
			//printf("Error 026\n");
		}

		if (xQueueReceive(queue_kalibracija1_max, &max, pdMS_TO_TICKS(20)) != pdTRUE) { // ceka vrednosti sa senzora 1
			//printf("Error 027\n");
		}

		if (cc == (uint8_t)0x0d) {  // ako je u promeljivu cc stigao karakter 0x0d, to je signal za kraj poruke i vrednost dobijena sa kanala 0 se moze obradjivati
			senzor1 = atof(rastojanje_kanal0); // pomocu funkcije atof, pretvaramo string u double
			printf("LEVI : %u \n", (unsigned)senzor1);
			br_karaktera = 0; // resetujemo broj karaktera na 0, da bi mogli uspesno da obradjuemo naredne poruke
			if (xQueueSend(queue_senzor1, &senzor1, 0U) != pdTRUE) {
				//printf("Error 028\n");
			}
			if (xQueueSend(queue_senzor1, &senzor1, 0U) != pdTRUE) {
				//printf("Error 029 \n");
			}

			

			kalibracija1_local = (double)100 * (senzor1 - min) / (max - min);  // racunamo kalibraciju, i smestamo je u promenljivu kalibracija1
			if (xQueueSend(queue_kalibracija1, &kalibracija1_local, 0U) != pdTRUE) {// vrednost kalibracije 1 saljemo u queue_kalibracija1, ovaj queue kasnije receivujemo 
				//printf("Error 030\n");                                                      //u Serijska_stanje_task da bi mogli na serijskoj da ispisujemo na serijskoj trenutno stanje kalibracije1
			}

			
			if (xQueueSend(queue_kalibracija3, &kalibracija1_local, 0U) != pdTRUE) {// ovaj queue receivujemo u tasku LED_bar_Task, koji nam sluzi za generisanje signala(blinkanje dioda odredjenom frekvecijom)
				//printf("Error 031\n");
			}

			
		}
		else {
			rastojanje_kanal0[br_karaktera++] = cc; // redom iz cc smestamo karakter po karakter u niz rastojanje_kanala dok ne stigne 0x0d 
		}


	}
}

static void Primio_kanal_1(const void* pvParameters) //prijem sa kanala 1 tj. senzor 2
{
	double senzor2 = 0; // ista pojasnjenja kao kod Primio_kanal_0 za sve promenljive
	uint8_t cc = 0;
	double kalibracija2_local = 0;
	double min = 20, max = 100;
	uint8_t br_karaktera = 0;
	uint8_t rastojanje_kanal1[6] = { 0 };
	const char rastojanje_kanal1_radi[6] = { "32" }; // POTREBNO DA BI POKAZALO DA ZEZA ATOF I DA BI SE POKAZALO DA RADI KOD

	for (;;) {
		if (xSemaphoreTake(RXC_BS_1, portMAX_DELAY) != pdTRUE) {
			//printf("Error 032\n");
		}


		if (get_serial_character(COM_CH1, &cc) != 0) {
			//printf("Error 033\n");
		}
		
		if (xQueueReceive(queue_kalibracija2_min, &min, pdMS_TO_TICKS(20)) != pdTRUE) { // ceka vrednosti sa senzora 2
			//printf("Error 034\n");
		}

		if (xQueueReceive(queue_kalibracija2_max, &max, pdMS_TO_TICKS(20)) != pdTRUE) { // ceka vrednosti sa senzora 2
			//printf("Error 035\n");
		}

		if (min > max) {

			printf("LOSE KALIBRISAN SISTEM, min mora biti manja vrednost - kalibrisati ponovo\n");
		}
		
		if (cc == (uint8_t)0x0d) {
			senzor2 = atof(rastojanje_kanal1);
			printf("DESNI : %u \n", (unsigned)senzor2);
			if (xQueueSend(queue_senzor2, &senzor2, 0U) != pdTRUE) {
				//printf("Error 069\n");
			}
			if (xQueueSend(queue_senzor2, &senzor2, 0U) != pdTRUE) {
				//printf("Error 070\n");
			}

			br_karaktera = 0;
			kalibracija2_local = (double)100 * (senzor2 - min) / (max - min);
			if (xQueueSend(queue_kalibracija2, &kalibracija2_local, 0U) != pdTRUE) {
				//printf("Error 071\n");
			}

			if (xQueueSend(queue_kalibracija4, &kalibracija2_local, 0U) != pdTRUE) {
				//printf("Error 072\n");
			}


		}
		else {
			rastojanje_kanal1[br_karaktera++] = cc;
		}

	}
}

static void SerialReceive_Task(void* pvParameters) //kanal 2, prima komandnu rijec koja se zavrsava karakterom 13(0x0d)
{
	uint8_t r_point = 0;  //za pomeranje kroz niz r_buffer
	uint8_t r_buffer[12]; //smestamo primljeno komandu
	uint8_t start_local = 0; //treba nam za START/STOP 
	uint8_t startovanje = 0; // moze i bez nje ali onda nesto zeza
	uint8_t cc = 0;  // ovde se smesta podatak od 2 bajta 
	uint8_t duzina_primljene_rijeci = 0; //duzinu primljene reci smestamo ovde, da kasnije mozemo proveriti koja je rec stigla
	double senzor1_local = 0, senzor2_local = 0;
	double min_local = 20, max_local = 100;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BS_2, portMAX_DELAY) != pdTRUE) { // ceka na serijski prijemni interapt
			//printf("Error 036\n");
		}

		

		if (get_serial_character(COM_CH2, &cc) != 0) {//ucitava primljeni karakter u promenjivu cc
			//printf("Error 037\n");
		}
		

		        

		if (xQueueReceive(stanje_sistema, &startovanje, pdMS_TO_TICKS(20)) != pdTRUE) {// ovo nam treba zbog START/STOP
			//printf("Error 038\n");
		}

		if (xQueueReceive(queue_senzor1, &senzor1_local, pdMS_TO_TICKS(20)) != pdTRUE) { // ceka vrednosti sa senzora 1
			//printf("Error 039\n");
		}

		if (xQueueReceive(queue_senzor2, &senzor2_local, pdMS_TO_TICKS(20)) != pdTRUE) { // ceka vrednosti sa senzora 1
			//printf("Error 040\n");
		}

		start_local = startovanje;

		if (cc == (uint8_t)0x0d) // oznaciti kraj poruke i ako je kraj, preko reda poslati informacije o poruci i restartovati ovaj taks
		{
			duzina_primljene_rijeci = r_point;
			if (xQueueSend(serijska_prijem_niz, &r_buffer, 0U) != pdTRUE) {
				//printf("Error 041\n");
			}

			// saljemo niz u queue koji kasnije koristimo za ispis
			if (xQueueSend(serijska_prijem_duzina, &r_point, 0U) != pdTRUE) {
				//printf("Error 042\n");
			}

			// saljemo duzinu tog niza, isto nam treba za ispis
			r_point = 0; // reset na nulu, da bi mogli uspesno ponovo da obradjujemo primljene poruke
		}
		else if (r_point < (uint8_t)R_BUF_SIZE)// pamti karaktere prije FF 
		{
			r_buffer[r_point++] = cc; // sve iz cc smestamo u niz r_buffer
		}
		//ovim if-om proveravamo da li je stigla prava rec, takodje proveravamo da li sistem vec upaljen,
		//ako jeste da se ne pali opet(apsurd), zato u uslovu stoji da je start==0, jer sistem treba da je prvo logicno UGASEN da bi se 
		//upalio
		if ((duzina_primljene_rijeci == (uint8_t)sizeof("START") - (uint8_t)1) && (strncmp(r_buffer, ("START"), duzina_primljene_rijeci) == (uint8_t)0) && start_local == (uint8_t)0) {

			if (set_LED_BAR(1, 0x01) != 0) {// palimo indikacionu diodu, drugi stubac, prva od dole.
				//printf("Error 043\n");
			}
			printf("START je ok uneto \n"); //ispisujemo na terminal da je korisnik uneo dobru komandu
			start_local = 1; // palimo sistem
		}

		//Slicna prica i sa ovim else if, samo se ovde proverava stop, i proveramo da li je sistem upaljen da bi ga gasili

		else if ((duzina_primljene_rijeci == (uint8_t)sizeof("STOP") - (uint8_t)1) && (strncmp(r_buffer, ("STOP"), duzina_primljene_rijeci) == (uint8_t)0) && start_local == (uint8_t)1) {

			if (set_LED_BAR(1, 0x00) != 0) {//gasimo indikacionu diodu
				//printf("Error 044\n");
			}
			printf("STOP je ok uneto \n");//obavestavamo korisnika da je ukucao dobru komandu
			start_local = 0; // gasimo sistem

		}

		else if ((duzina_primljene_rijeci == (uint8_t)sizeof("KALIBRACIJA1100") - (uint8_t)1) && (strncmp(r_buffer, ("KALIBRACIJA1100"), duzina_primljene_rijeci) == (uint8_t)0) && start_local == (uint8_t)1) {

			max_local = senzor1_local;

			if (xQueueSend(queue_kalibracija1_max, &max_local, 0U) != pdTRUE) {
				//printf("Error 045\n");
			}
			
			printf("Dobro uneseno 100 procenata - senzor 1 \n");//obavestavamo korisnika da je ukucao dobru komandu
			
		}

		else if ((duzina_primljene_rijeci == (uint8_t)sizeof("KALIBRACIJA100") - (uint8_t)1) && (strncmp(r_buffer, ("KALIBRACIJA100"), duzina_primljene_rijeci) == (uint8_t)0) && start_local == (uint8_t)1) {

			min_local = senzor1_local;

			if (xQueueSend(queue_kalibracija1_min, &min_local, 0U) != pdTRUE) {
				//printf("Error 046\n");
			}

			printf("Dobro uneseno 0 procenata - senzor 1 \n");//obavestavamo korisnika da je ukucao dobru komandu

		}

		else if ((duzina_primljene_rijeci == (uint8_t)sizeof("KALIBRACIJA2100") - (uint8_t)1) && (strncmp(r_buffer, ("KALIBRACIJA2100"), duzina_primljene_rijeci) == (uint8_t)0) && start_local == (uint8_t)1) {

			max_local = senzor2_local;

			if (xQueueSend(queue_kalibracija2_max, &max_local, 0U) != pdTRUE) {
				//printf("Error 047\n");
			}

			printf("Dobro uneseno 100 procenata - senzor 2 \n");//obavestavamo korisnika da je ukucao dobru komandu

		}

		else if ((duzina_primljene_rijeci == (uint8_t)sizeof("KALIBRACIJA200") - (uint8_t)1) && (strncmp(r_buffer, ("KALIBRACIJA200"), duzina_primljene_rijeci) == (uint8_t)0) && start_local == (uint8_t)1) {

			min_local = senzor2_local;

			if (xQueueSend(queue_kalibracija2_min, &min_local, 0U) != pdTRUE) {
				//printf("Error 048\n");
			}

			printf("Dobro uneseno 0 procenata - senzor 2 \n");//obavestavamo korisnika da je ukucao dobru komandu

		}

if (xQueueSend(stanje_sistema, &start_local, 0U) != pdTRUE) {// saljemo u queue stanje_sistema koja je komanda aktivirana, ovaj queue sluzi za task Serijska_stanje_task
	//printf("Error 049\n");
}

if (xQueueSend(seg7_auto_queue, &start_local, 0U) != pdTRUE) {// saljemo u queue seg7_auto_queue koja je komanda aktivirana, ovaj queue sluzi za Seg7_ispis(nisam siguran da je to bas ime taska)
                                                             //razlog zasto imam dva reda, za istu stvar, je sinhronizacija izmedju taskova koja se poremeti ako koristim isti queue.
	//printf("Error 050\n");
}

	}
}



static void Seg7_ispis_task(void* pvParameters) { // TASK ZA ISPIS NA SEG7 DISPLEJU
										   // NA SEG7 DISPLEJU ISPISUJEMO, NA PRVOM SEGMETU 0(AKO JE SISTEM UGASEN), ILI 1 (AKO JE SISTEM AKTIVIRAN)
											// SLEDECA TRI SEGMENTA, ISPISUJEMO VREDNOST SA SENZORA 1(NEKALIBRISANU)
											// SLEDECA TRI SEGMENTA, ISPISUJEMO VREDNOST SA SENZORA 2

	double senzor1_local = 0, senzor2_local = 0;  // promenljive za smestanje vrednost sa senzora
	uint8_t start_local = 0; // promeljiva za smestanje vrednosti start/stop

	for (;;) {
		if (xSemaphoreTake(seg7_ispis, portMAX_DELAY) != pdTRUE) {// ceka semafor, osvezava se displej svakih 200ms
			//printf("Error 051\n");
		}

		

		if (xQueueReceive(seg7_auto_queue, &start_local, pdMS_TO_TICKS(20)) != pdTRUE) { //ceka komandu za start
			//printf("Error 052\n");
		}
		

		
		if (xQueueReceive(queue_senzor1, &senzor1_local, pdMS_TO_TICKS(20)) != pdTRUE) { // ceka vrednosti sa senzora 1
			//printf("Error 053\n");
		}


		
		if (xQueueReceive(queue_senzor2, &senzor2_local, pdMS_TO_TICKS(20)) != pdTRUE ) { // ceka vrednosti sa senzora 2
			//printf("Error 054\n");
		}


		

		if (start_local == (uint8_t)1) { //na prvu cifru ispisuje 1 ako je rezim rada start, a 0 ako je stop
			if (select_7seg_digit(0) != 0) {
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[1]) != 0) {
				printf("Problem\n");
			}

			if (select_7seg_digit(1) != 0) { // sledece tri cifre vrednost sa senzora 1
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[(uint16_t)senzor1_local / (uint16_t)100]) != 0) {
			printf("Problem\n");
			}
			if (select_7seg_digit(2) != 0) {
			printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[((uint16_t)senzor1_local / (uint16_t)10) % (uint16_t)10]) != 0) {
			printf("Problem\n");
			}
			if (select_7seg_digit(3) != 0) {
			printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[(uint16_t)senzor1_local % (uint16_t)10]) != 0) {
			printf("Problem\n");
			}
			if (select_7seg_digit(4) != 0) {
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[(uint16_t)senzor2_local / (uint16_t)100]) != 0) {
			printf("Problem\n");
			}
			if (select_7seg_digit(5) != 0) {
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[((uint16_t)senzor2_local / (uint16_t)10) % (uint16_t)10]) != 0) {
				printf("Problem\n");
			}
			if (select_7seg_digit(6) != 0) {
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[(uint16_t)senzor2_local % (uint16_t)10]) != 0) {
			printf("Problem\n");
			}

		}

		else {
			if (select_7seg_digit(0) != 0) {
				printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0){
			printf("Problem\n");
			}
			if (select_7seg_digit(1) != 0){
				printf("Problem\n");
			}
			if (set_7seg_digit(hexnum[0]) != 0) {
				printf("Problem\n");
			}

			if (select_7seg_digit(2) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}

			if (select_7seg_digit(3) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}

			if (select_7seg_digit(4) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}

			if (select_7seg_digit(5) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}

			if (select_7seg_digit(6) != 0) {
			printf("Problem\n");
			}

			if (set_7seg_digit(hexnum[0]) != 0) {
			printf("Problem\n");
			}
			
			
		}

	}

	
}

static void Serijska_stanje_task(void* pvParameters) { /*formiramo niz za redovan ispis stanja sistema i saljemo pomocu reda poruku i duzinu poruke
												tasku za ispis na serijsku*/
	uint8_t pomocni_niz[60] = { 0 }; // POMOCNI NIZ U KOJI SMESTAMO KOMPLETNU PORUKU ZA SLANJE NA SERIJSKU KANALA 2
	uint8_t duzina_niza_ispis = 0; // POMOCNA PROMENLJIVA POMOCU KOJE SALJEMO DUZINU TOG NIZA
	uint8_t start_local = 0; // smesta se start/stop
	double kalibracija1_local = 0, kalibracija2_local = 0;  // u ove promenljive smestamo vrednosti kalibracije

	for (;;) {
		if (xSemaphoreTake(serijska_stanje, portMAX_DELAY) != pdTRUE) { // UZIMA SEMAFOR SVAKIH 5 SEKUNDI 
			//printf("Error 055\n");
		}

		if (xQueueReceive(queue_kalibracija1, &kalibracija1_local, pdMS_TO_TICKS(20)) != pdTRUE) { // RISIVUJE VREDNOST KALIBRACIJE 1
			//printf("Error 056\n");
		}
	

		if (xQueueReceive(queue_kalibracija2, &kalibracija2_local, pdMS_TO_TICKS(20)) != pdTRUE) { // RISIVUJE VREDNOST KALIBRACIJE 2
			//printf("Error 057\n");
		}
		

		
		if (xQueueReceive(stanje_sistema, &start_local, pdMS_TO_TICKS(20)) != pdTRUE) { // RISIVUJE KOMANDU START/STOP
			//printf("Error 058\n");
		}
		strcpy(pomocni_niz,"Stanje: "); // PRVO U NIZ pomocni_niz SMESTAMO(KOPIRAMO) "Stanje:" 
		duzina_niza_ispis = (uint8_t)sizeof("Stanje: ") - (uint8_t)1; // OVDE U PROMENLJIVU DUZINA_NIZA_ISPIS SMESTAMO KOLIKO JE TRENUTNO DUGACAK NIZ POMOCNI_NIZ

		if (start_local == (uint8_t)1) { //AKO JE AKTIVAN START NA Stanje: nadovezi START
			strcat(pomocni_niz, "START");//POMOCU FUNKCIJE strcat mi cemo "START" da 'priljubimo' uz Stanje: i to ce izgledati na terminalu Stanje:START
			duzina_niza_ispis += (uint8_t)sizeof("START") - (uint8_t)1; //u duzinu_niza_ispis sumiramo i broj karaktera komande START
		}
		else {
			strcat(pomocni_niz, "STOP"); // identicno sa komandama iznad samo sa STOP
			duzina_niza_ispis += (uint8_t)sizeof("STOP") - (uint8_t)1;
		}

		if (start_local == (uint8_t)1) {// prikazuje vrednosti kalibracije

			strcat(pomocni_niz, ", K1:"); //U POMOCNI NIZ dodajemo K1:
			duzina_niza_ispis += (uint8_t)sizeof(", K1:") - (uint8_t)1; // SUMIRAMO U DUZINU
			pomocni_niz[duzina_niza_ispis++] = (uint8_t)kalibracija1_local / (uint8_t)100 + (uint8_t)'0'; // I ONDA SE REDOM POMERAMO PO POMOCNOM NIZU I SMESTAMO CIFRU PO CIFRU KALIRBACIJE 1
			pomocni_niz[duzina_niza_ispis++] = (((uint8_t)kalibracija1_local / (uint8_t)10) % (uint8_t)10) + (uint8_t)'0';
			pomocni_niz[duzina_niza_ispis++] = (uint8_t)kalibracija1_local % (uint8_t)10 + (uint8_t)'0';


			strcat(pomocni_niz, ", K2:"); // ISTA PRICA KAO ZA K1
			duzina_niza_ispis += (uint8_t)sizeof(", K2:") - (uint8_t)1;
			pomocni_niz[duzina_niza_ispis++] = (uint8_t)kalibracija2_local / (uint8_t)100 + (uint8_t)'0';
			pomocni_niz[duzina_niza_ispis++] = (((uint8_t)kalibracija2_local / (uint8_t)10) % (uint8_t)10) + (uint8_t)'0';
			pomocni_niz[duzina_niza_ispis++] = (uint8_t)kalibracija2_local % (uint8_t)10 + (uint8_t)'0';

		}

		if (xQueueSend(serijska_ispis_queue, &pomocni_niz, 0U) != pdTRUE) { 
			//printf("Error 059\n");
		}

		if (xQueueSend(serijska_ispis_duzina, &duzina_niza_ispis, 0U) != pdTRUE) { 
			//printf("Error 060\n");
		}

		if (send_serial_character(COM_CH2, 13) != 0) {
			//printf("Error 061\n");
		}


	}
}

/* OPC - ON INPUT CHANGE - INTERRUPT HANDLER */      // POGLEDATI NA FREERTOS.ORG
static uint32_t OnLED_ChangeInterrupt(void)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &higher_priority_task_woken) != pdTRUE) {

		//printf("Error 062\n");
	}

	portYIELD_FROM_ISR((uint32_t)higher_priority_task_woken);
}


/* TBE - TRANSMISSION BUFFER EMPTY - INTERRUPT HANDLER */
static uint32_t prvProcessTBEInterrupt(void)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (get_TBE_status(0) != 0) {
		if (xSemaphoreGiveFromISR(TBE_BS_0, &higher_priority_task_woken) != pdTRUE) {

			//printf("Error 063\n");
		}
	}
	if (get_TBE_status(1) != 0) {
		if (xSemaphoreGiveFromISR(TBE_BS_1, &higher_priority_task_woken) != pdTRUE) {

			//printf("Error 064\n");
		}
	}
	if (get_TBE_status(2) != 0) {
		if (xSemaphoreGiveFromISR(TBE_BS_2, &higher_priority_task_woken) != pdTRUE) {

			//printf("Error 065\n");
		}
	}
	
	portYIELD_FROM_ISR((uint32_t)higher_priority_task_woken);
}


/* RXC - RECEPTION COMPLETE - INTERRUPT HANDLER */
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (get_RXC_status(0) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_0, &higher_priority_task_woken) != pdTRUE) {

			//printf("Error 066\n");
		}
	}
	
	if (get_RXC_status(1) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_1, &higher_priority_task_woken) != pdTRUE) {

			//printf("Error 067\n");
		}
	}

	if (get_RXC_status(2) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_2, &higher_priority_task_woken) != pdTRUE) {

			//printf("Error 068\n");
		}
	}


	portYIELD_FROM_ISR((uint32_t)higher_priority_task_woken);
}


/* PERIODIC TIMER CALLBACK */
static void TimerCallback(TimerHandle_t per_TimerHandle)
{
	uint32_t timer_id;
	timer_id = (uint32_t)pvTimerGetTimerID(per_TimerHandle);

	if (timer_id == (uint32_t)1) {
		if (xTimerIsTimerActive(per_TimerHandle1) != pdFALSE) {
			if (xTimerStop(per_TimerHandle1, 0) != pdPASS) {
				printf("Zeza tajmer\n");
			}
			return;
		}
		else {
			if (xTimerStart(per_TimerHandle1, 0) != pdPASS) {
				printf("Zeza tajmer\n");
			}
			return;
		}
	}
	if (xSemaphoreGive(seg7_ispis) != pdTRUE) {
		printf("Error 069\n");
	}
	
	//update-ovanje displeja svakih 80ms

}


static void ispis_tajmer_callback(TimerHandle_t ispis_podaci_tajmer)
{
	uint32_t timer_id1;
	timer_id1 = (uint32_t)pvTimerGetTimerID(ispis_podaci_tajmer1);

	if (timer_id1 == (uint32_t)1) {
		if (xTimerIsTimerActive(ispis_podaci_tajmer1) != pdFALSE) {
			if (xTimerStop(ispis_podaci_tajmer1, 0) != pdPASS) {
				printf("Zeza tajmer\n");
			}
			return;
		}
		else {
			if (xTimerStart(ispis_podaci_tajmer1, 0) != pdPASS) {
				printf("Zeza tajmer\n");
			}
			return;
		}
	}
	if (xSemaphoreGive(serijska_stanje) != pdTRUE) {
		printf("Error 070\n");
	}
	
	    // kanal 2 ispis svakih 5000ms

}


/* MAIN - SYSTEM STARTUP POINT */
void main_demo(void)
{
	/* Inicijalizacija periferija */
	if (init_7seg_comm() != 0) {
		
		printf("Neuspesna inicijalizacija\n");
	}

	if (init_LED_comm() != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_uplink(COM_CH) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}
	
	if (init_serial_downlink(COM_CH) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_uplink(COM_CH1) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_downlink(COM_CH1) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_uplink(COM_CH2) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}

	if (init_serial_downlink(COM_CH2) != 0) {

		printf("Neuspesna inicijalizacija\n");
	}
	

	/* ON INPUT CHANGE INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);

	/* Create LED interrapt semaphore */
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();
	if (LED_INT_BinarySemaphore == NULL) {
		printf("Greska prilikom kreiranja");
	}
	

	/* create a timer task */
	per_TimerHandle1 = xTimerCreate("Timer", pdMS_TO_TICKS(80), pdTRUE, NULL, TimerCallback);
	
	if (per_TimerHandle1 == NULL) {
		printf("Greska prilikom kreiranja\n");
	}
	if (xTimerStart(per_TimerHandle1, 0) != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	
	
	
	ispis_podaci_tajmer1 = xTimerCreate("Timer2", pdMS_TO_TICKS(5000), pdTRUE, NULL, ispis_tajmer_callback);

	if (ispis_podaci_tajmer1 == NULL) {
		printf("Greska prilikom kreiranja\n");
	}
	if (xTimerStart(ispis_podaci_tajmer1, 0) != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	
	
	/* SERIAL TRANSMITTER TASK */
	BaseType_t status1;
	status1 = xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);
	if (status1 != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	/* SERIAL RECEIVER TASK */
	BaseType_t status2;
	status2 = xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);
	if (status2 != pdPASS) {
		printf("Greska prilikom kreiranja");
	}
	/* Create TBE semaphore - serial transmit comm */
	TBE_BS_0 = xSemaphoreCreateBinary();
	if (TBE_BS_0 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	TBE_BS_1 = xSemaphoreCreateBinary();
	if (TBE_BS_1 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	TBE_BS_2 = xSemaphoreCreateBinary();
	if (TBE_BS_2 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/* Create RXC semaphore - serial transmit comm */
	RXC_BS_0 = xSemaphoreCreateBinary();
	if (RXC_BS_0 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	RXC_BS_1 = xSemaphoreCreateBinary();
	if (RXC_BS_1 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	RXC_BS_2 = xSemaphoreCreateBinary();
	if (RXC_BS_2 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/* Ostali semafori */

	serijska_stanje = xSemaphoreCreateBinary(); // SEMAFOR KOJI SE GIVUJE SVAKIH 5 SEKUNDI ZA ISPIS STANJA NA KANALU 2
	if (serijska_stanje == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	seg7_ispis = xSemaphoreCreateBinary(); // KREIRAM SEMAFOR KOJI SE POSLE TAJMEROM GIVUJE SVAKIM 200mS TASKU SEG7 DISPLEJ, OSVEZAVAMO DISPEL SVAKIH 200mS
	if (seg7_ispis == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	/* SERIAL TRANSMISSION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);

	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

	/* Kreiranje redova za komunikaciju izmedju taskova */


	serijska_ispis_queue = xQueueCreate(3, sizeof(uint8_t[60])); //red za skladistenje poruke za ispis
	if (serijska_ispis_queue == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	serijska_ispis_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine rijeci
	if (serijska_ispis_duzina == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	seg7_auto_queue = xQueueCreate(2, sizeof(uint8_t)); // SMESTAMO KOMANDU START/STOP
	if (seg7_auto_queue == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	
	serijska_prijem_niz = xQueueCreate(3, sizeof(uint8_t[12])); //red za skladistenje primljene rijeci
	if (serijska_prijem_niz == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	serijska_prijem_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine primljene rijeci preko serijske komunikacije
	if (serijska_prijem_duzina == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/*QUEUE-OVI ZA POTREBE SKLADISTANJE VREDNOSTI SA SENZORA I KALIBRACIJU*/

	queue_senzor1 = xQueueCreate(2, sizeof(double)); //red za primanje vrijednosti sa senzora 1
	if (queue_senzor1 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_senzor2 = xQueueCreate(2, sizeof(double)); //red za primanje vrijednosti sa senzora 2
	if (queue_senzor2 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_kalibracija1 = xQueueCreate(2, sizeof(double));//red za primenje kalibrisane vrednosti sa senzora 1
	if (queue_kalibracija1 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_kalibracija2 = xQueueCreate(2, sizeof(double));//red za primenje kalibrisane vrednosti sa senzora 2
	if (queue_kalibracija2 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_kalibracija3 = xQueueCreate(2, sizeof(double));//red za primenje kalibrisane vrednosti sa senzora 1
	if (queue_kalibracija3 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	queue_kalibracija4 = xQueueCreate(2, sizeof(double));////red za primenje kalibrisane vrednosti sa senzora 2
	if (queue_kalibracija4 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	queue_kalibracija1_max = xQueueCreate(2, sizeof(double));//red za primanje maksimuma senzora 1 za potrebe kalibracije
	if (queue_kalibracija1_max == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	queue_kalibracija1_min = xQueueCreate(2, sizeof(double));//red za primanje minimuma senzora 1 za potrebe kalibracije
	if (queue_kalibracija1_min == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	queue_kalibracija2_max = xQueueCreate(2, sizeof(double));//red za primanje maksimuma senzora 2 za potrebe kalibracije
	if (queue_kalibracija2_max == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	queue_kalibracija2_min = xQueueCreate(2, sizeof(double));//red za primanje minimuma senzora 2 za potrebe kalibracije
	if (queue_kalibracija2_min == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/*QUEUE U KOJI SE SMEŠTAJU START/STOP KOMANDE*/
	stanje_sistema = xQueueCreate(1, sizeof(uint8_t));//PRIMA 0 ILI 1 U ZAVISNOSTI DA LI SISTEM AKTIVAN (1) ILI UGAŠEN (0)
	if (stanje_sistema == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}
	// KREIRAMO TASKOVE

	BaseType_t status;
	status = xTaskCreate(led_bar_tsk, "ST", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)SERVICE_TASK_PRI, NULL);// TASK ZA PROVERU DA LI JE PRITISNUT PREKIDAČ
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}

	status = xTaskCreate(Primio_kanal_0, "kanal0", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAl_REC_PRI, NULL);//TASK ZA PRIJEM SA KANALA 0
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}

	status = xTaskCreate(Primio_kanal_1, "kanal1", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAl_REC_PRI, NULL);//TASK ZA PRIJEM SA KANALA 1
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}

	status = xTaskCreate(LED_bar_Task1, "LEtsk1", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)OBRADA_TASK_PRI, NULL);  //TASK ZA BLINKANJE DIODA 1
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}

	status = xTaskCreate(LED_bar_Task2, "LEtsk2", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)OBRADA_TASK_PRI, NULL); //TASK ZA BLINKANJE DIODA 2
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}

	status = xTaskCreate(Seg7_ispis_task, "Seg_7", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)SERVICE_TASK_PRI, NULL);//TASK ZA ISPIS NA SEG7 DISPLEJ
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}

	status = xTaskCreate(Serijska_stanje_task, "Stanje", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)OBRADA_TASK_PRI, NULL);// TASK ZA ISPIS NA KANALU 2
	if (status != pdPASS) {
		printf("Greska prilikom kreiranja\n");
	}
	

	
	vTaskStartScheduler(); // OVAJ RADI KAD SVI OSTALI NISU AKTIVNI


	for (;;);
}

