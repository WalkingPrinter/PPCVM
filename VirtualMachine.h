#pragma once

#define PPC_STOP_INSTRUCTION    0x00000000
#define PPC_SC_INSTRUCTION      0x44000002  // System Call
#define PPC_RFI_INSTRUCTION     0x4C000064  // Return from Interrupt

class Memory {
public:
    explicit Memory(DWORD size);
    ~Memory();

    bool read32(DWORD address, DWORD& value);
    bool write32(DWORD address, DWORD value);
    bool isValidAddress(DWORD address) const;

private:
    std::vector<BYTE> data_;
};

class Registers {
public:
    Registers();

    void reset();
    DWORD getGPR(DWORD index) const;
    void setGPR(DWORD index, DWORD value);

    DWORD getCR() const { return cr_; }
    void setCR(DWORD value) { cr_ = value; }

    DWORD getLR() const { return lr_; }
    void setLR(DWORD value) { lr_ = value; }

private:
    static const DWORD NUM_GPRS = 32;
    DWORD gprs_[NUM_GPRS];
    DWORD cr_;  // Condition Register
    DWORD lr_;  // Link Register
};

class PowerPCVM {
public:
    PowerPCVM();
    ~PowerPCVM();

    bool initialize(DWORD memory_size);
    void reset();
    bool executeInstruction();
    bool loadProgram(const std::vector<DWORD>& program, DWORD address);

    // Accessors
    Memory* getMemory() { return memory_; }
    Registers* getRegisters() { return registers_; }

private:
    Memory* memory_;
    Registers* registers_;
    DWORD pc_; // Program Counter
    bool running_;

    // Instruction decoding helpers
    DWORD fetchInstruction();
    void decodeAndExecute(DWORD instruction);

    // Helper methods
    void updateCR0(DWORD result) {
        DWORD cr = registers_->getCR();
        cr &= ~(0xF0000000);  // Clear CR0 field (first 4 bits)

        if ((INT)result < 0) {
            cr |= 0x80000000;  // Set LT bit (Negative)
        }
        else if ((INT)result > 0) {
            cr |= 0x40000000;  // Set GT bit (Positive)
        }
        else {
            cr |= 0x20000000;  // Set EQ bit (Zero)
        }

        // Note: We don't set the SO (Summary Overflow) bit here
        // In a full implementation, i should handle this

        registers_->setCR(cr);
    }

    // Basic PPC instruction implementations
    void executeADD(DWORD instruction);
    void executeSUB(DWORD instruction);
    void executeMULLW(DWORD instruction);
    void executeDIVW(DWORD instruction);
    void executeLWZ(DWORD instruction);
    void executeSTW(DWORD instruction);
    void executeB(DWORD instruction);
    void executeBCOND(DWORD instruction);
};