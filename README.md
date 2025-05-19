# Embarcatech_FilaRTOS

Repositório criado para a tarefa relacionada à construção de uma simulação de estação de monitoramento de enchentes utilizando os conceitos de multitarefas e filas no freeRTOS.

Este projeto implementa a estação de monitoramento utilizando os seguintes periféricos da placa **BitDogLab**:

- LED RGB  
- Matriz de LEDs WS2812  
- Display OLED SSD1306  
- Buzzer
- Joystick

O software é desenvolvido com o uso das *tasks* do **FreeRTOS**, permitindo o gerenciamento paralelo de cada periférico. Ao todo, são implementadas **cinco tasks** distintas, uma para cada periférico.
Cada task consome dados de uma **fila** que é populada com as leituras obtidas do Joystick pelo do ADC.

### Funcionamento

1. **Nível Médio**
   - Ativado se o nível de água for maior que 40% ou volume de chuva for maior que 50%.
   - O display exibe a mensagem: `"Atenção Nível Alto"`, além dos níveis de interesse (água e chuva)
   - O buzzer bipa intermitentemente em baixa frequência enquanto os níveis se mantiverem no intervalo
   - A matriz acende um quadrado amarelo de tamanho 3x3
   - O Led RGB acende em amarelo

2. **Nível Alto**  
   - Ativado se o nível de água for maior que 70% ou volume de chuva for maior que 80%.
   - O display exibe a mensagem: `"Perigo! Nível Extremo"`, além dos níveis de interesse (água e chuva)
   - O buzzer bipa intermitentemente em alta frequência enquanto os níveis se mantiverem no intervalo
   - A matriz acende um quadrado vermelho de tamanho 5x5
   - O Led RGB acende em vermelho

3. **Nível Normal**
   - Ativado se o nível de água for menor que 40% e o volume de chuva for menor que 50%.
   - O display exibe a mensagem: `"Nível seguro"`, além dos níveis de interrese (água e chuva)
   - O buzzer, a matriz e o led rgb se mantém desligados
     
# Instruções de Compilação

Para compilar o código, são necessárias as seguintes extensões:

  - Raspberry Pi Pico SDK

  - CMake

  - FreeRTOS instalado

Após instalá-las, basta importar através da extensão do raspberry pi, mudar o diretorio do FreeRTOS no arquivo CMakelists.txt para o diretorio correspondente à sua instalação 
e construir (buildar) o projeto utilizando o CMake.

# Vídeo Demonstrativo

https://youtu.be/Iow1baTyMZI

