# 1inch_test_task
## Реализация
В основе реализации лежат две задачи из файла `main.c`:

Управление приёмом данных от прерывания SPI и последующая передача при помощи UART:
```c
void UartTxTask(void *argument)
{
  for(;;)
  {
    uint8_t data;
    xStreamBufferReceive(xSpiRxStreamBuffer, &data, 1, portMAX_DELAY);
    LL_USART_TransmitData8(USART2, data);
    LL_USART_EnableIT_TXE(USART2);
    xSemaphoreTake(xUartTxDoneSemaphore, portMAX_DELAY);
  }
}
```

Управление приёмом данных от прерывания UART и последующая передача при помощи SPI:
```c
void SpiTxTask(void *argument)
{
  for(;;)
  {
    uint8_t data;
    // We have to send data even there is no input data to be able to get data from SPI slave
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
    if (xStreamBufferReceive(xUartRxStreamBuffer, &data, 1, 0)) {
      LL_SPI_TransmitData8(SPI2, data);
    } else {
      LL_SPI_TransmitData8(SPI2, 0);
    }
    LL_SPI_EnableIT_TXE(SPI2);
    xSemaphoreTake(xSpiTxDoneSemaphore, portMAX_DELAY);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
  }
}
```

И две функции прерываний из файла `stm32f4xx_it.c`:

Обработчик прерываний для SPI:
```c
void SPI2_IRQHandler(void)
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  static bool prev_data_is_null = true;
  if (LL_SPI_IsActiveFlag_RXNE(SPI2)) {
    uint8_t data = LL_SPI_ReceiveData8(SPI2);
    if ((data != 0) || ((data == 0) && (prev_data_is_null == false))) {
      xStreamBufferSendFromISR(xSpiRxStreamBuffer, &data, 1, &pxHigherPriorityTaskWoken);
    }
    prev_data_is_null = !data;
  } else {
	  assert(LL_SPI_IsActiveFlag_TXE(SPI2));
	  LL_SPI_DisableIT_TXE(SPI2);
	  xSemaphoreGiveFromISR(xSpiTxDoneSemaphore, &pxHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
```
Обработчик прерываний для UART:
```c
void USART2_IRQHandler(void)
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  if (LL_USART_IsActiveFlag_RXNE(USART2)) {
    uint8_t data = LL_USART_ReceiveData8(USART2);
    xStreamBufferSendFromISR(xUartRxStreamBuffer, &data, 1, &pxHigherPriorityTaskWoken);
  } else {
    assert(LL_USART_IsActiveFlag_TXE(USART2));
    LL_USART_DisableIT_TXE(USART2);
    xSemaphoreGiveFromISR(xUartTxDoneSemaphore, &pxHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
```
## Способ взаимодействия
Для получения данных от периферии используется прерывание `RXNE` (SPI и UART). Принятые данные складываются в соответсвующий Stream Buffer (`xSpiRxStreamBuffer` если данные пришли от SPI и `xUartRxStreamBuffer` если данные пришли от UART).

В задачах же мы ожидаем приеёма данных в соответствующем буфере и по принятии отправляем в другой периферийный модуль (UART RX -> SPI TX, SPI RX -> UART TX).

Так как в основе реализации лежит библиотека Low Level Driver от ST, то необходимо самостоятельно отслеживать отправку данных. Для этого реализиваны бинарные семафоры, которые сигнализируют задачам, что данные были отправленые. Устанавливаются же семафоры в функции прерываний по приходу события `TXE`.

Прерывание SPI несколько отличается от прерывания UART, так как там необходимо сделать "предобработку" данных перед сохранением данных в буфер. Если мы получаем `0` в качестве данных, то это может означать, что данных у SPI Slave устройства нет и этот байт не нужно сохранять. Исключением является случай, когда после не нулевого байта следует нулевой, то его нужно сохранить, так как он будет является признаком конца строки.

Задача SPI несколько отличается от задача UART, так как нам необходимо постоянно отправлять данные Slave устройству даже, если у нас нет никаких данных для отправки. Это необходимо, так как для чтения данных Slave устройства должен генерироваться CLK импульс и он генерируется при передаче данных.

## Улучшение

Задачу UART можно удалить и использовать только задачу SPI для отправки данных. 

Отправка же данных по UART будет осуществляться напрямую из обработчика прерываний. SPI обработчик по принятии байта данных в буфер будет проверять включено ли прерывание передачи UART и будет его включать, если оно выключено.
```c
void SPI2_IRQHandler(void)
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  static bool prev_data_is_null = true;
  if (LL_SPI_IsActiveFlag_RXNE(SPI2)) {
    uint8_t data = LL_SPI_ReceiveData8(SPI2);
    if ((data != 0) || ((data == 0) && (prev_data_is_null == false))) {
      xStreamBufferSendFromISR(xSpiRxStreamBuffer, &data, 1, &pxHigherPriorityTaskWoken);
      if (!LL_USART_IsEnabledIT_TXE(USART2)) {
      	LL_USART_EnableIT_TXE(USART2);
      }
    }
    prev_data_is_null = !data;
  } else {
	  assert(LL_SPI_IsActiveFlag_TXE(SPI2));
	  LL_SPI_DisableIT_TXE(SPI2);
	  xSemaphoreGiveFromISR(xSpiTxDoneSemaphore, &pxHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
```

Обработчик прерываний UART будет по приходу события `TXE` брать данные из буфера RX SPI и отправлять данные. Если данных в буфере нет, то отключит прерывание `TXE`.
```c
void USART2_IRQHandler(void)
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
  if (LL_USART_IsActiveFlag_RXNE(USART2)) {
    uint8_t data = LL_USART_ReceiveData8(USART2);
    xStreamBufferSendFromISR(xUartRxStreamBuffer, &data, 1, &pxHigherPriorityTaskWoken);
  } else {
    assert(LL_USART_IsActiveFlag_TXE(USART2));
    uint8_t data;
    if (xStreamBufferReceiveFromISR(xSpiRxStreamBuffer, &data, 1, &pxHigherPriorityTaskWoken)) {
      LL_USART_TransmitData8(USART2, data);
    } else {
      LL_USART_DisableIT_TXE(USART2);
    }
  }
  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}
```

В основе этого способа лежит факт того, что прерывания имеют одинаковый приоритет и из обработчика SPI можно безопасно проверять статус и включать прерывания UART и, соответственно, обработчик UART может безопасно отключать свои прерывания.   
