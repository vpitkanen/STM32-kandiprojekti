# STM32 Kandiprojekti

---

## Yhteenveto

Projektin tarkoituksena oli tutkia kahden eri STM32-mallin suorituskykyä kuvansiirrossa ja rivien summalaskennassa.  
Toteutus hyödyntää PSSI-tiedonsiirtoväylää, DMA-siirtoa, keskeytyksiä ja HAL-kirjaston tarjoamia laitetason funktioita.
---

Repository sisältää testauskoodin, joka vastaanottaa **16-bittisiä, 128x128 pikselin kuvia anturilta** ja suorittaa **rivien summalaskennan** kuville STM32-mikrokontrollerilla.

---

## Koodiversiot

### `main_H723ZG.c`
Sisältää **STM32-H723ZG** -mallille toteutetun pääohjelman.

### `main_U575ZI.c`
Sisältää **STM32-U575ZI** -mallille toteutetun pääohjelman.

---

## Ohjelman toiminta

Ohjelma käynnistyy alustamalla **PSSI-väylän** ja nostamalla **ready-ohjauspinnin** ylös ilmoittamaan, että mikrokontrolleri on valmis vastaanottamaan dataa.  
Samalla käynnistetään ajastin.

1. Sensori päivittää datan, jonka jälkeen mikrokontrolleri **lukee sen ja tallentaa muistipuskuriin DMA:n avulla**.  
2. Kun siirto on valmis ja puskuri täynnä, **DMA lähettää keskeytyssignaalin**.  
3. Keskeytyskäsittelijässä:
   - otetaan ajastimesta siirron kestoaika  
   - suoritetaan datan prosessointifunktio (rivien summalaskenta)  
   - mitataan prosessoinnin kesto  
   - nostetaan lippumuuttuja ilmoittamaan siirron valmistumisesta  
4. Pääohjelma (main) tarkistaa lipun tilan ja päättää:
   - aloitetaanko uusi siirto  
   - vai onko 10 kuvaa jo käsitelty  

Kun 10 kuvaa on käsitelty, ohjelma **laskee siirtoihin ja prosessointiin kuluneen kokonaisajan** ja **tulostaa tulokset terminaaliin**.

---

## Virrankulutusmittaus

Virrankulutusmittauksessa ohjelmaa muokattiin siten, että **ready-ohjauspinni nostetaan heti ylös uuden siirron aloitusta varten**, kun edellinen on valmis.  


---

## Toteutustiedot

- Ohjelmointikieli: **C (HAL-kirjasto, STM32CubeIDE)**
- Väylä: **PSSI**
- Kuvadata: **16-bittinen, 128×128 pikseliä**
- Kellotaajuus: molemmat mikrokontrollerit toimivat **maksimikellotaajuudella**
- Toiminnot: DMA, ajastimet, keskeytyskäsittely, GPIO-signaalit

---

## Projektirakenne
STM32-Kandiprojekti/
├─ main_H723ZG.c
├─ main_U575ZI.c
└─ README.md

