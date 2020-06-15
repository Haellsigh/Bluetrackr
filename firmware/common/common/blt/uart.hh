#include <string_view>

#include <blt/hal_include.hh>

namespace blt::uart {

void init(UART_HandleTypeDef* huart);
void transmit(uint8_t data);
void transmit(uint8_t* data, uint16_t len);
void transmit(std::string_view sv);

}