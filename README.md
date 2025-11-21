TP de Synthèse – Autoradio

Kenny & Mantou

1. Démarrage
1.1 Création du projet
Nous travaillons avec la NUCLEO-L476RG dans STM32CubeIDE.
Tous les périphériques matériels ont été laissés avec leur configuration par défaut, et la BSP n’a pas été activée afin de conserver un projet minimal.

1.2 Test de la LED LD2

Un premier test fonctionnel a validé le pilotage de la LED LD2 via un simple toggle.
Cela permet de confirmer la bonne initialisation du GPIO et du processus de compilation/flash.

1.3 Test de l’USART2 (ST-Link interne)

L’USART2 a été configuré afin d’utiliser la liaison série intégrée au ST-Link pour :

la communication avec un terminal série,

l’affichage des messages,

la future intégration du shell.

1.4 Fonction printf

Pour utiliser printf via l’UART, la redirection de la sortie standard a été implémentée via :

int __io_putchar(int chr)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
	return chr;
}

Le terminal série reçoit désormais les messages envoyés avec printf().

1.5 Activation de FreeRTOS

FreeRTOS a été activé dans STM32CubeIDE en mode CMSIS V1.

1.6 Mise en place du Shell
a)b) Shell dans une tâche FreeRTOS et en mode interruption

Shell exécuté dans une tâche dédiée avec une boucle de traitement.
![SC_shell](./Datasheets/SC_Shell.png)

c) Shell avec un driver structuré



2. GPIO Expander & VU-Mètre
2.1 Configuration
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