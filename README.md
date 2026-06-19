# Seguidor Luminico 
> **Asignatura:** Electrónica Digital [III] - Universidad Nacional de Córdoba
> **Integrantes:** > * Gutierrez Patricio
> * Matias Zaambellini
> **Profesor:** Marcos Blasco

---

## 🚀 1. Descripción General del Proyecto 
El proyecto consiste en un seguidor luminico, osea busca el sector con mayor cantidad de luz. Para esto se utilizan 4 LDR divididas por una superficie opaca para que pueda generar una sombra y mediante la deferencias de las resistencias LDRs hacer mover a dos servos uno con movimiento horizontal y otro con movimiento vertical. A su vez dispone de una alarma (utilizazndo dma y dac para producirla) mientras se encuentra en movimiento. Es posible tambien si se requiere de tomar la telemetria de los ldr, detener los servos, volverlos a arrancar y dejarlos en posicion defaullt.
El problema que resuelve a mayor escala seria por ejemplo,redirigir un panel solar a lo largo del dia para  obtener la mayor cantidad de luz. Va dirigido principalmente al sector energetico.


### 🎯 Alcances del Proyecto (¿Qué hace y qué NO hace el sistema?)
Delimiten claramente los objetivos alcanzados para la entrega final:
* **El sistema SÍ es capaz de:** Medir la cantidad de luz recibida por resistencias LDR en tiempo real, actibar una alarma mientras se mueve y mandar telemetria a travez del UART 
* **El sistema NO incluye (Fuera de alcance):** movimiento mayores a 180 grados de los servos. 

### ⏩ Posibles Etapas Siguientes (Líneas Futuras)
Planteen cómo escalaría este desarrollo en una versión 2.0 o en un ámbito profesional:
En el ambito profesional se utilizarian motores mucho mas potentes, mayor movimiento, la utilizacion de paneles solares y componentes mucho mas precisos que unas LDR.
Se podria diseñar una pagina web para recolectar datos de telemetria, temperatura de motores y para controlar a larga distancia. 

---

## 📐 2. Arquitectura del Sistema: Hardware y Software (Común)

### 🔌 Hardware & Interconexión

* **Esquemático del Circuito:** 

  `![Esquemático Completo](hardware/esquematico.png)`
* **Descripción del Circuito y Consideraciones de Diseño:** Breve explicación de las etapas (ej: acoplamiento de señales, protecciones inductivas, filtrado, etc.).

### 💻 Arquitectura de Software (Firmware)
* **Diagrama de Flujo o Máquina de Estados:** *[Inserte aquí la imagen del diagrama que explique el lazo principal o el comportamiento del sistema]*
  `![Diagrama de Flujo / Máquina de Estados](docs/diagrama_software.png)`

---

## ⚡ 3. Especificaciones Eléctricas, Alimentación y Entorno (Específico por Asignatura)

### 🔌 Parámetros de Alimentación y Consumo (Común a ambas materias)
* **Tensión de operación del sistema:** 5V (para servos y buzzer) / 3.3V (resto de componentes)
* **Método de alimentación:** USB y fuente continua
* **Consumo estimado o medido:** * En modo activo (máxima carga, servos moviéndose): ~300 mA a 450 mA
  * En modo reposo (servos en posición fija, midiendo sensores): ~70 mA a 100 mA



### 📌 [OPCIÓN B: Solo para alumnos de Electrónica Digital III (Cortex-M / ARM)]
* **IDE y SDK:**  MCUXpresso IDE v25.6.136.
* **Microcontrolador Principal:**  NXP LPC1769 .
* **Bibliotecas de Terceros y Versiones:** solo drivers de la catedra.
* **Periféricos Avanzados Utilizados:**  NVIC, DMA, SysTick, DAC, Timer, Uart y ADC.
* **Estrategia de Concurrencia:** Expliquen la arquitectura elegida: [Ej: Bare-metal con máquina de estados cooperativa / RTOS (FreeRTOS) detallando las tareas creadas y sus prioridades].

---

## 🔄 4. Proceso de Integración y Desarrollo (Común)
Describan cronológicamente cómo fueron sumando y testeando las diferentes partes del proyecto (enfoque modular de ingeniería).

* **Etapa 1 (Validación inicial):** verificacion de funcionamiento de servos, ldr y todo el resto de componentes
* **Etapa 2 (Adquisición/Comunicación):** implementacion de configuraciones de modulos y despues implementacion de funciones necesarias 
* **Etapa 3 (Integración lógica):** implementacion de codigo para testeo 
* **Etapa 4 (Sistema Completo):** soldarura de componentes, calibracion de LDR y servos , pruebas de estres 

---

## 📊 5. Ensayos, Pruebas y Resultados (Común)
Demuestren con datos empíricos que el sistema funciona correctamente. **Es obligatorio incluir registro visual**.
Para hacer que el DAC y DMA funcionen correctamente se realizo la experiencia por separado, haciendo uso de un osciloscopio para ver que la señal sea la correcta.
se utilizo una linterna y poca luz para tomar medidas con el tester.
* **Pruebas Funcionales Realizadas:** Detallen los ensayos (Ej: "Se inyectó una señal controlada para medir la precisión del ADC...").
* **Evidencia Fotográfica y Gráficos:** * *Capturas de instrumental:* [Insertar capturas de Osciloscopio, Analizador Lógico o Terminal Serie]
  * *Foto del Prototipo Real:* [Insertar foto del hardware final cableado/armado en funcionamiento]

---

## 📂 6. Estructura del Repositorio (Común)
El repositorio debe mantener obligatoriamente la siguiente estructura limpia (¡Recuerden configurar correctamente el `.gitignore` para no subir carpetas temporales como `Debug/`, `Release/` o archivos `.p1` / `.d`!).

```text
├── firmware/          # Código fuente del proyecto (MPLABX / MCUXpresso / STM32Cube)
│   ├── src/           # Archivos de código (.c)
│   └── inc/           # Archivos de cabecera (.h)
├── hardware/          # Archivos de diseño (KiCad/Altium), esquemáticos en PDF/Imagen y BOM
├── docs/              # Datasheets clave, imágenes del README, notas de aplicación
└── README.md          # Este archivo de presentación