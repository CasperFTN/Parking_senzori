# Parking senzori

Svrha ovog projekta je simulacija parking senzora u automobilu. Ceo kod je pisan u Visual Studio uz pomoć FREERTOS biblioteka.

## Periferije potrebne za testiranje

Prvo je potrebno pokrenuti sve potrebne periferije. Od periferija treba LED prvi stubac ulazni i tri izlazna npr. rBGG, sedmodegmenti se pokrece sa 7 cifara, UniCom se pokrece na kanalima 0,1,2.
Na kanalima 0 i 1 postavlja se trigger na XYZ.

Ove periferije je moguće pokrenuti preko batch fajlova i to:

- AdvUniCom.bat 
- LED_bars_plus.bat
- Seg7_Mux.bat

Takođe moguće ih je pokrenuti i preko jednog .bat fajla pod nazivom:

- otvori_sve_potrebne_periferije.bat

NAPOMINJEMO DA JE PRVO POTREBNO KORIGOVATI OTVORI_SVE_POTREBNE_PERIFERIJE.BAT FAJL DA BI SE LEPO NAMESTILE PUTANJE DO SVIH POJEDINAČNIH .BAT
FAJLOVA.

Od periferija ne treba vise nista.

Periferije nisu uključene u ove fajlove, podrazumeva se da ko testira već poseduje periferije dok su batch fajlovi tu da omoguće lako
uključivanje periferija prilikom svakog testiranja.

## Kako se testira

Sistem se moze aktivirati pritiskom na prekidac( prvi stubac LED_bar-a, skroz donja dioda ) ili preko serijske upisom START\od na kanalu 2.

Sistem se gasi preko prekidaca ( isto prvi stubac LED_bar-a ) ili preko serijske upisom STOP\0d na kanalu 2.

Na terminalu se ispisuje vrednost dobijena sa levog i desnog senzora kao i kalibracija (NEMA, UDALJENA, BLISKA ili KONTAKT detekcija) za svaki senzor pojedinacno.

Kalibracija se vrsi upisom preko serijske i to:
- KALIBRACIJA1100\0d - vrednost dobijena sa senzora 1 postaje maksimum preko koga se racuna kalibracija tj. ta vrednost postaje 100%
- KALIBRACIJA100\0d - vrednost dobijena sa senzora 1 postaje minimum preko koga se racuna kalibracija tj. ta vrednost postaje 0%
- KALIBRACIJA2100\0d - vrednost dobijena sa senzora 2 postaje maksimum preko koga se racuna kalibracija tj. ta vrednost postaje 100%
- KALIBRACIJA100\0d - vrednost dobijena sa senzora 2 postaje minimum preko koga se racuna kalibracija tj. ta vrednost postaje 0%

U zavisnosti od kalibrisanih vrednosti senzora 1, gornje cetiri diode treceg stubca sa leva ce blinkati ( zavisi u kojoj zoni se nalazi objekat koji je detektovan senzorom ).
Ista stvar vazi i za kalibrisane vrednosti sa senzora 2, samo ce sada blinkati diode CETVRTOG stubca sa leva.


Na sedmosegmentnom displeju se ispisuju redom: prvi cifra ako je 0 ( sistem ugasen ), ako je 1( sistem upaljen ), 
naredne tri cifre predstavljaju vrednost sa senzora 1 ( nekalibrisanu ), naredne tri cifre predstavljaju vrednost sa senzora 2 ( nekalibrisanu ).
                                                

Ostale informacije o kodu mogu se naći u fajlu "Parking senzori.docx".

## Opis taskova

U kodu je realizovano 9 taskova i to:

- ``` static void led_bar_tsk(const void* pvParameters); ```
- ``` static void Primio_kanal_0(const void* pvParameters); ```
- ``` static void Primio_kanal_1(const void* pvParameters); ```
- ``` static void LED_bar_Task1(const void* pvParameters); ```
- ``` static void LED_bar_Task2(const void* pvParameters); ```
- ``` static void SerialSend_Task(const void* pvParameters); ```
- ``` static void SerialReceive_Task(void* pvParameters); ```
- ``` static void Seg7_ispis_task(void* pvParameters); ```
- ``` static void Serijska_stanje_task(void* pvParameters); ```

led_bar_tsk služi za očitavanje vrednosti prekidača sa prvog stupca LED bara ( crveni stub, jedini ulazni ) i da na osnovu njega manipuliše
promenljivom start_local koja govori da li je sistem pokrenut.

Primio_kanal_0 služi da pokupi podatak koji je stigao na kanal 0 serijske ( tj. na LEVI senzor ), da ga ispiše na terminal i da ga preko 
atof funkcije pretvori u double vrednost koju smeštamo u promenljivu senzor1. Takođe, ovaj task računa kalibracionu vrednost senzora 1
i smešta ga u promenljivu kalibracija1_local.

Primio_kanal_1 služi da pokupi podatak koji je stigao na kanal 1 serijske ( tj. na DESNI senzor ), da ga ispiše na terminal i da ga preko 
atof funkcije pretvori u double vrednost koju smeštamo u promenljivu senzor2. Takođe, ovaj task računa kalibracionu vrednost senzora 2
i smešta ga u promenljivu kalibracija2_local.

LED_bar_Task1 uzima vrednost kalibracije dobijenu u tasku Primio_kanal_0 i na osnovu njega blinka diode trećeg stupca LED bara i to:
blinka LED frekvencijom 100Hz ako je kalibraciona vrednost između 50 i 100%, blinka LED frekvencijom 200Hz ako je kalibraciona vrednost između 0 i 50%
i konstanto svetli frekvencijom 200Hz ako je manje od 0.
Takođe ispisuje na terminal NEMA DETEKCIJE, UDALJENA DETEKCIJA, BLISKA DETEKCIJA ili KONTAKT DETEKCIJA u zavisnosti od kalibracione vrednosti.

LED_bar_Task2 uzima vrednost kalibracije dobijenu u tasku Primio_kanal_1 i na osnovu njega blinka diode četvrtog stupca LED bara i to:
blinka LED frekvencijom 100Hz ako je kalibraciona vrednost između 50 i 100%, blinka LED frekvencijom 200Hz ako je kalibraciona vrednost između 0 i 50%
i konstanto svetli frekvencijom 200Hz ako je manje od 0.
Takođe ispisuje na terminal NEMA DETEKCIJE, UDALJENA DETEKCIJA, BLISKA DETEKCIJA ili KONTAKT DETEKCIJA u zavisnosti od kalibracione vrednosti.

SerialSend_Task služi da ispiše na serijsku ( tj. na kanal 2 ) niz koji je dobijen od Serijska_stanje_task. Ovo se ispisuje na svakih 5 sekundi i ovaj ispis
predstavlja stanje sistema ( upaljen ili ugašen i kalibrisane vrednosti senzora ako je upaljen ).

SerialReceive_Task služi da primi komandu sa serijske kanala 2 i da je uporedi sa unapred poznatim komandama START\od, STOP\0d, KALIBRACIJA100\0d,
KALIBRACIJA1100\0d, KALIBRACIJA200\0d i KALIBRACIJA2100\0d. Ukoliko je uneta komanda neka od poznatih onda izvrši ono što ta komanda radi a ako
nije uneta dobra komanda ispiši grešku.

Seg7_ispis_task služi za ispisivanje da li je sistem upaljen ( prva cifra 1 ) ili ugašen ( prva cifra 0 ) kao i nekalibrisane vrednosti senzora 1 i 2.
Tj. druga, treća i četvrta cifra ispisuju nekalibrisanu vrednost levog senzora a preostale 3 cifre ispisuju nekalibrisanu vrednost desnog senzora.
Ovo se izvršava svakih 80ms.

Serijska_stanje_task služi da napravi niz koji opisuje stanje sistema i koji koristi task SerialSend_Task da taj niz ispiše na kanal 2 serijske.
