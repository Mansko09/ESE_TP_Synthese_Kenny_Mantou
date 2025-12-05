# TP de Synthèse – Autoradio  
**Kenny & Mantou**

---

## 1. Démarrage

### 1.1 Création du projet
Nous travaillons avec la **NUCLEO-L476RG** dans **STM32CubeIDE**.  
Tous les périphériques ont été laissés avec leur configuration par défaut et la **BSP n’a pas été activée** afin d’obtenir un projet minimal.

---

### 1.2 Test de la LED LD2
Un premier test fonctionnel a validé le toggle simple de la LED LD2.  
Cela permet de confirmer :
- l’initialisation correcte du GPIO,
- la chaîne compilation → flash → exécution.

---

### 1.3 Test de l’USART2 (ST-Link interne)
L’USART2 a été configuré pour :
- communiquer avec un terminal série,
- afficher des messages,
- intégrer un shell dans la suite.

---

### 1.4 Fonction `printf`
Redirection du `printf` vers l’USART2 :

```c
int __io_putchar(int chr)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
    return chr;
}
```
Le terminal série reçoit désormais les messages envoyés avec printf().

### 1.5 Activation de FreeRTOS

FreeRTOS a été activé dans STM32CubeIDE en mode CMSIS V1.

### 1.6 Mise en place du Shell
a)b) Shell dans une tâche FreeRTOS et en mode interruption

Shell exécuté dans une tâche dédiée avec une boucle de traitement.
![SC_shell](./Datasheets/SC_Shell.png)

c) Shell avec un driver structuré



## 2. GPIO Expander & VU-Mètre
### 2.1 Configuration
1) Référence du GPIO Expander

Le GPIO Expander utilisé est un MCP23S17 (Microchip).
La datasheet a été étudiée pour déterminer les registres et la configuration SPI.

2) SPI utilisé sur le STM32

La communication avec le MCP23S17 utilise le SPI3 du STM32L476.

3) Configuration STM32CubeIDE

Éléments configurés :

Mode : Full-Duplex Master

Vitesse : 8 Mbit/s (limitation MCP23S17 = 10 MHz)

Data size : 8 bits

First bit : MSB First

Clock polarity/phase : Mode 0

Chip Select (nCS) : PB7 en Output Push-Pull

### 2.2 Tests
<img src="https://github.com/user-attachments/assets/cd7800a6-c962-4c1d-abff-19b98b26e7c1" width="300"/>

<img src="https://github.com/user-attachments/assets/3fe16403-e0ff-4fff-a76d-b7a572f37050" width="300"/>



## 3. Le CODEC Audio SGTL5000
### 3.1 Configuration préalable 
1) L'I2C utilise les pins PB10(SCL) et PB11(SDA). Cela correspond à l'I2C2. 

### 3.2 Configuration du CODEC par l'I2C 

1)
<img src="https://github.com/user-attachments/assets/c3e19b24-1870-4d2f-a8de-61d600c8f278" width="300"/>

2)
<img src="https://github.com/user-attachments/assets/452dc4d6-affd-472a-83a6-81098b02c1ec" width="300"/>

3) 
<img src="https://github.com/user-attachments/assets/ad56656d-0c34-4ddb-978e-e0df89cce364" width="300"/>

### 3.3 Signaux I2S
2) On démarre la réception et la transmission sur l'I2S avec le SAI: 
<img src="https://github.com/user-attachments/assets/13dac0d9-5874-46e8-8dcb-af52840721e9" width="300"/>

### 3.4 Génération de signal audio 
1) On génère un signal triangulaire
<img src="https://github.com/user-attachments/assets/f5921383-9da3-4b63-b9fa-4c4d19f4d414" width="300"/>


## 5 - Filtre RC 
### 1. Équation différentielle du filtre RC

La loi des mailles donne :

$$
C \frac{dV_{out}(t)}{dt} = \frac{V_{in}(t) - V_{out}(t)}{R}
$$

En isolant l’entrée, on obtient l’équation classique :

$$
V_{in}(t) = RC \frac{dV_{out}(t)}{dt} + V_{out}(t)
$$

Donc sous la forme demandée :

$$
V_{in} = X \cdot \frac{dV_{out}}{dt} + Y \cdot V_{out}
$$

avec :

- **X = RC**
- **Y = 1**

### 2. Discrétisation 

On approxime la dérivée par une différence finie :

$$
\frac{dV_{out}}{dt} \approx f_s \left( V[n] - V[n-1] \right)
$$

où :
- $f_s$ = fréquence d’échantillonnage
- $T_s = \frac{1}{f_s}$ = période d’échantillonnage

On remplace dans l’équation :

$$
V_{in}[n] = RC \cdot f_s \left( V[n] - V[n-1] \right) + V[n]
$$

Développement :

$$
V_{in}[n] = (RC \cdot f_s + 1) V[n] - (RC \cdot f_s) V[n-1]
$$

On isole $V[n]$ (sortie courante) :

$$
V[n] = \frac{V_{in}[n] + (RC \cdot f_s) V[n-1]}{RC \cdot f_s + 1}
$$

### 3. Forme finale de l’équation de récurrence

On met sous la forme demandée :

$$
V[n] = \frac{A \cdot V_{in}[n] + B \cdot V[n-1]}{D}
$$

En identifiant les coefficients :

- **A = 1**
- **B = RC \cdot f_s**
- **D = RC \cdot f_s + 1**

Comme la constante de temps $RC = \frac{1}{2\pi f_c}$ (où $f_c$ = fréquence de coupure), on peut aussi écrire :

$$
B = \frac{f_s}{2\pi f_c} \quad ; \quad D = 1 + \frac{f_s}{2\pi f_c}
$$

### 4. Nombre de cycles MCU par échantillon à 48 kHz

Fréquence d’échantillonnage : **48 kHz**  
Période d’échantillon :

$$
T_s = \frac{1}{48000} \approx 20.833\,\mu s
$$

Notre microcontrôleur tourne à **80 MHz** :

$$
80\,000\,000 \times 20.833 \times 10^{-6} \approx 1667 \text{ cycles par échantillon}
$$


à fc=10kHz

<img src="https://github.com/user-attachments/assets/0748afe7-c9f9-40e8-8a8a-97ce3254b336" width="300"/>

On modifie avec le shell fc:

<img src="https://github.com/user-attachments/assets/ec383a14-df64-45b0-8b9e-15f31a94555b" width="300"/>

<img src="https://github.com/user-attachments/assets/371aae22-2623-4f7e-9ac1-07de2e018c02" width="300"/>

