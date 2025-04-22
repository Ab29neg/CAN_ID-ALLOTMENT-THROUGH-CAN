#include "stm32f1xx.h"

// Global variable to store the current CAN ID of the STM32 board
volatile uint32_t current_can_id = 0x123; // Default CAN ID

// Simple delay function (not precise, just a loop-based delay)
void delay(volatile uint32_t count) {
    while (count--);
}

// Function to initialize CAN peripheral and GPIO
void can_init(void) {
    // Enable clock for GPIOA (used for CAN pins)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR|=RCC_APB1ENR_CAN1EN;

    // Configure PA12 (CAN_TX) as Alternate Function Push-Pull
    GPIOA->CRH &= ~(0xF << 16); // Clear bits
    GPIOA->CRH |=  (0xB << 16); // MODE=11 (50MHz), CNF=10 (AF PP)

    // Configure PA11 (CAN_RX) as Input Floating
    GPIOA->CRH &= ~(0xF << 12); // Clear bits
    GPIOA->CRH |=  (0x4 << 12); // MODE=00, CNF=01 (Floating input)

    // Enable CAN1 clock
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    // Exit sleep mode
    CLEAR_BIT(CAN1->MCR, CAN_MCR_SLEEP);
    while ((CAN1->MSR & CAN_MSR_SLAK) != 0U);

    // Enter initialization mode
    CAN1->MCR |= CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK));


    CAN1->BTR = 0x001c0000;  // TS1=13, TS2=2, SJW=1, Prescaler=4

    // Leave initialization mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while (CAN1->MSR & CAN_MSR_INAK);

    // Initialize CAN filter to accept all standard IDs
    CAN1->FMR |= CAN_FMR_FINIT;  // Enter filter init mode

    CAN1->FA1R |= 1 << 0;        // Activate filter 0
    CAN1->FS1R |= 1 << 0;        // Set filter scale to 32-bit
    CAN1->FM1R &= ~(1 << 0);     // Identifier Mask mode
    CAN1->sFilterRegister[0].FR1 = 0x00000000;  // ID filter
    CAN1->sFilterRegister[0].FR2 = 0x00000000;  // ID mask
    CAN1->FFA1R &= ~(1 << 0);    // Assign filter to FIFO0

    CAN1->FMR &= ~CAN_FMR_FINIT; // Exit filter init mode

}

// Function to send a CAN message using the current CAN ID
void can_send(uint32_t id, uint8_t* data, uint8_t len) {
    // Wait for an empty transmission mailbox
    while ((CAN1->TSR & CAN_TSR_TME0) == 0);

    // Set up the ID (standard 11-bit ID format)
    CAN1->sTxMailBox[0].TIR = (id << 21); // STDID in bits [31:21]
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_RTR; // Data frame
    CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_IDE; // Standard ID mode

    // Set the Data Length Code (0–8 bytes)
    CAN1->sTxMailBox[0].TDTR = len & 0xF;

    // Load first 4 bytes into the low register
    CAN1->sTxMailBox[0].TDLR =
        (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | (data[0]);

    // Load next 4 bytes into the high register
    CAN1->sTxMailBox[0].TDHR =
        (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | (data[4]);

    // Request transmission
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}

// Function to check for received CAN messages and extract data
int can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    // Check if there's a message in FIFO0
    if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0)
        return 0; // No message available

    // Get the message ID (Standard ID)
    *id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;

    // Get data length (DLC)
    *len = CAN1->sFIFOMailBox[0].RDTR & 0xF;

    // Read data bytes
    uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;

    for (int i = 0; i < 4; i++) {
        data[i] = (rdlr >> (8 * i)) & 0xFF;
        data[i + 4] = (rdhr >> (8 * i)) & 0xFF;
    }

    // Release the FIFO (make it ready for the next message)
    CAN1->RF0R |= CAN_RF0R_RFOM0;

    return 1; // Message received
}


int main(void) {
    can_init(); // Initialize CAN and GPIOs

    uint8_t tx_data[8] = {0}; // Data to transmit
    uint8_t rx_data[8];       // Buffer for received data
    uint8_t len;              // Length of received message
    uint32_t rx_id;           // ID of received message

    while (1) {
        // Send a heartbeat/status message using current CAN ID
        tx_data[0] = 0xAB;  // Example payload
        tx_data[1] = 0xCD;
        can_send(current_can_id, tx_data, 2);

        // Check for incoming messages
        if (can_receive(&rx_id, rx_data, &len)) {
            // If message with ID 0x7FF and magic bytes {0xDE, 0xAD}, update CAN ID
            if (rx_id == 0x7FF && len >= 4 &&
                rx_data[0] == 0x01 && rx_data[1] == 0x23) {

                // Extract new CAN ID from next two bytes
                uint32_t new_id = (rx_data[2] << 8) | rx_data[3];
                current_can_id = new_id & 0x7FF; // Mask to 11-bit CAN ID
            }
        }

        delay(1000000); // Wait ~1 second before sending next message
    }
}
