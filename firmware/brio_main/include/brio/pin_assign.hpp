#ifndef BRIO_PIN_ASSIGN_HPP_
#define BRIO_PIN_ASSIGN_HPP_
namespace brio {
namespace pin_assign {
constexpr int kUartTx1 = 1;
constexpr int kSpeeker = 2;
constexpr int kUartRx1 = 3;
constexpr int kEmg = 4;
constexpr int kStepperDriverCS = 5;
constexpr int kHSPI_MISO = 12;
constexpr int kHSPI_MOSI = 13;
constexpr int kHSPI_SCK = 14;
constexpr int kWS2815 = 15;
constexpr int kUartTx2 = 16;
constexpr int kUartRx2 = 17;
constexpr int kVSPI_SCK = 18;
constexpr int kVSPI_MISO = 19;
constexpr int kVSPI_MOSI = 23;
constexpr int kLCD_Reset = 25;
constexpr int kLCD_CS = 26;
constexpr int kTOUCH_CS = 27;
constexpr int kTOUCH_IRQ = 32;
constexpr int kLCD_BACKLIGHT = 33;
}  // namespace pin_assign
}  // namespace brio
#endif  // BRIO_PIN_ASSIGN_HPP_
