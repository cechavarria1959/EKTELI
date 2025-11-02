#include <SPI.h>
#include <mcp2515.h>
#include <stdint.h>

#define CAN_DOWNLOAD_STD_ID (0x230u)
#define CAN_ACK_ID (0x555)

#define BUFFER_SIZE (1024) 

struct can_frame canMsg;
MCP2515 mcp2515(10);
int pushButton = PC13;

volatile bool buffer_available = false;
char buffer[BUFFER_SIZE];
size_t buf_size;

int last_added = 0;
int last_printed = 0;

int tail = 0;

int errors=0;

uint8_t ack_flag = 0;

void irqHandler()
{
    uint8_t irq = mcp2515.getInterrupts();

    if(irq & MCP2515::CANINTF_RX0IF || irq & MCP2515::CANINTF_RX1IF)
    {
        // while(mcp2515.checkReceive() == true)
        // {
            if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
            {
                // if(canMsg.can_id == CAN_ACK_ID)
                // {
                //     Serial.println("ack!");
                //     ack_flag = 1;
                //     return;
                // }

                uint8_t dlc = canMsg.can_dlc;
                
                if(last_added + dlc >= BUFFER_SIZE)
                {
                    int tmp_size = BUFFER_SIZE - last_added;
                    memcpy(&buffer[last_added], canMsg.data, tmp_size);
                    memcpy(&buffer[0], canMsg.data, dlc - tmp_size);
                    last_added = dlc - tmp_size;
                }
                else
                {
                    memcpy(&buffer[last_added], canMsg.data, dlc);
                    last_added += dlc;
                }
                
                buf_size += dlc;
                buffer_available = true;
            }
        // }
    }
    else
    {
        errors=1;
    }
}
uint32_t time_send;
void setup()
{
    Serial.begin(115200);

    Serial.println("---UART-CAN bridge---\n\n");
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    attachInterrupt(2, irqHandler, FALLING);

    time_send = millis();
}
uint8_t buffer2[1030];
uint32_t packet_size = 0;
uint32_t punterobuffer = 0;
int bytestotales = 0;

bool read_serial()
{
    bytestotales = 0;
    packet_size = 1028;
    uint32_t timestart = millis();
    uint32_t timeout = 0;
    
    while(bytestotales < packet_size && timeout <= 500)
    {
        if(Serial.available())
        {
            Serial.readBytes(&buffer2[bytestotales++], 1);
        }

        timeout = millis() - timestart;
    }

    packet_size = 0;
    switch (buffer2[0])
    {
        case 1:
            packet_size = 128 + 4;
            break;
        case 2:
            packet_size = 1024 + 4;
            break;
        case 4:
            break;
        case 0x18:
            packet_size = 2;
            break;
        case 0x41:
        case 0x61:
            break;
        default:
            break;
    }

    send_one_char(buffer2[0]);

    if(packet_size > 1)
    {
        bytestotales--;
        return true;
    }
    else
        return false;
}

void loop()
{
    int bytes = Serial.available();

    if(bytes)
    {
        if(read_serial())
            send_multiple_bytes(bytes);
    }

    if (buffer_available)
    {
        while(buf_size > 0)
        {
            Serial.write(&buffer[last_printed], 1);
            buf_size--;
            last_printed++;
            if(last_printed == BUFFER_SIZE)
                last_printed = 0;
        }

        buffer_available = false;
    }

    if(errors)
    {
        mcp2515.clearRXnOVRFlags();
        mcp2515.clearERRIF();
        mcp2515.clearMERR();
        mcp2515.clearRXnOVR();
        mcp2515.clearInterrupts();
        mcp2515.clearTXInterrupts();
    }
}

void send_one_char(int chr)
{
    struct can_frame cantx;

    cantx.can_id = CAN_DOWNLOAD_STD_ID;
    cantx.can_dlc = 1;
    memcpy(cantx.data, &chr, 1);

    delay(1);

    while(mcp2515.sendMessage(MCP2515::TXB0, &cantx) != MCP2515::ERROR_OK);
}


void send_multiple_bytes(int tx_count)
{
    struct can_frame cantx;

    punterobuffer = 1;
    tx_count = bytestotales;
    while (tx_count > 0u)
    {
        size_t bytes_to_copy = (tx_count >= 8u) ? 8u : tx_count;

        memcpy(cantx.data, &buffer2[punterobuffer], bytes_to_copy);

        punterobuffer += bytes_to_copy;
        tx_count -= bytes_to_copy;

        cantx.can_id = CAN_DOWNLOAD_STD_ID;
        cantx.can_dlc = bytes_to_copy;        

        delay(10);
        while(mcp2515.sendMessage(MCP2515::TXB0, &cantx) != MCP2515::ERROR_OK);
    }
}
