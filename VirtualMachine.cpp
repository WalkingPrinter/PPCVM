#include "../StdAfx.h"

PowerPCVM::PowerPCVM()
    : memory_(NULL)
    , registers_(NULL)
    , pc_(0)
    , running_(false)
{
}

PowerPCVM::~PowerPCVM() {
    delete memory_;
    delete registers_;
}

bool PowerPCVM::initialize(DWORD memory_size) {
    memory_ = new Memory(memory_size);
    registers_ = new Registers();
    reset();
    return true;
}

void PowerPCVM::reset() {
    pc_ = 0;
    running_ = false;
    registers_->reset();
}

DWORD PowerPCVM::fetchInstruction() {
    DWORD instruction;
    if (!memory_->read32(pc_, instruction)) {
        running_ = false;
        return PPC_STOP_INSTRUCTION;
    }

    // Check for special instructions first
    if (instruction == PPC_STOP_INSTRUCTION ||
        instruction == PPC_SC_INSTRUCTION ||
        instruction == PPC_RFI_INSTRUCTION) {
        running_ = false;
        return instruction;
    }

    pc_ += 4;
    return instruction;
}

bool PowerPCVM::loadProgram(const std::vector<DWORD>& program, DWORD address) {
    for (size_t i = 0; i < program.size(); ++i) {
        if (!memory_->write32(address + i * 4, program[i])) {
            return false;
        }
    }
    pc_ = address;
    running_ = true;
    return true;
}

bool PowerPCVM::executeInstruction() {
    if (!running_) return false;

    DWORD instruction = fetchInstruction();
    decodeAndExecute(instruction);
    return running_;
}

void PowerPCVM::decodeAndExecute(DWORD instruction) {
    // Check for special instructions first
    if (instruction == PPC_STOP_INSTRUCTION) {
        LogDebug("Executing STOP instruction");
        running_ = false;
        return;
    }

    // Basic instruction decoding based on primary opcode (bits 0-5)
    DWORD primary_opcode = (instruction >> 26) & 0x3F;

    LogDebug("Executing instruction 0x%08X (primary opcode: %d)", instruction, primary_opcode);

    switch (primary_opcode) {
    case 14: // addi
    {
        DWORD rd = (instruction >> 21) & 0x1F;
        DWORD ra = (instruction >> 16) & 0x1F;
        short imm = instruction & 0xFFFF;
        DWORD a = (ra == 0) ? 0 : registers_->getGPR(ra);
        registers_->setGPR(rd, a + imm);
        LogDebug("ADDI r%d, r%d, %d", rd, ra, imm);
    }
    break;

    case 31: // Extended opcode instructions
    {
        DWORD extended_opcode = (instruction >> 1) & 0x3FF;
        LogDebug("Extended opcode: %d", extended_opcode);
        switch (extended_opcode) {
        case 266: executeADD(instruction); break;
        case 40:  executeSUB(instruction); break;
        case 235: executeMULLW(instruction); break;
        case 491: executeDIVW(instruction); break;
        default:
            LogDebug("Unhandled extended opcode: %d", extended_opcode);
            running_ = false;
            break;
        }
    }
    break;

    case 32: executeLWZ(instruction); break;
    case 36: executeSTW(instruction); break;
    case 18: executeB(instruction); break;
    case 16: executeBCOND(instruction); break;

    default:
        LogDebug("Unhandled primary opcode: %d", primary_opcode);
        running_ = false;
        break;
    }
}

void PowerPCVM::executeADD(DWORD instruction) {
    DWORD rd = (instruction >> 21) & 0x1F;
    DWORD ra = (instruction >> 16) & 0x1F;
    DWORD rb = (instruction >> 11) & 0x1F;

    DWORD result = registers_->getGPR(ra) + registers_->getGPR(rb);
    registers_->setGPR(rd, result);
}

void PowerPCVM::executeSUB(DWORD instruction) {
    DWORD rd = (instruction >> 21) & 0x1F;
    DWORD ra = (instruction >> 16) & 0x1F;
    DWORD rb = (instruction >> 11) & 0x1F;

    DWORD result = registers_->getGPR(ra) - registers_->getGPR(rb);
    registers_->setGPR(rd, result);
}

void PowerPCVM::executeLWZ(DWORD instruction) {
    DWORD rd = (instruction >> 21) & 0x1F;
    DWORD ra = (instruction >> 16) & 0x1F;
    short displacement = instruction & 0xFFFF;

    DWORD address = registers_->getGPR(ra) + displacement;
    DWORD value;
    if (memory_->read32(address, value)) {
        registers_->setGPR(rd, value);
    }
    else {
        running_ = false;
    }
}

void PowerPCVM::executeSTW(DWORD instruction) {
    DWORD rs = (instruction >> 21) & 0x1F;
    DWORD ra = (instruction >> 16) & 0x1F;
    short displacement = instruction & 0xFFFF;

    DWORD address = registers_->getGPR(ra) + displacement;
    if (!memory_->write32(address, registers_->getGPR(rs))) {
        running_ = false;
    }
}

void PowerPCVM::executeMULLW(DWORD instruction) {
    DWORD rd = (instruction >> 21) & 0x1F;
    DWORD ra = (instruction >> 16) & 0x1F;
    DWORD rb = (instruction >> 11) & 0x1F;

    DWORD a = registers_->getGPR(ra);
    DWORD b = registers_->getGPR(rb);
    DWORD result = a * b;

    registers_->setGPR(rd, result);

    if (instruction & 0x1) {
        DWORD cr = registers_->getCR();
        cr &= ~(0xF0000000);  // Clear CR0

        if (result == 0) {
            cr |= 0x20000000;  // Set EQ bit
        }
        else if ((INT)result < 0) {
            cr |= 0x80000000;  // Set LT bit
        }
        else {
            cr |= 0x40000000;  // Set GT bit
        }

        registers_->setCR(cr);
    }
}

void PowerPCVM::executeDIVW(DWORD instruction) {
    DWORD rd = (instruction >> 21) & 0x1F;
    DWORD ra = (instruction >> 16) & 0x1F;
    DWORD rb = (instruction >> 11) & 0x1F;

    INT dividend = (INT)registers_->getGPR(ra);
    INT divisor = (INT)registers_->getGPR(rb);

    if (divisor == 0) {
        running_ = false;
        return;
    }

    if (dividend == -2147483648 && divisor == -1) {
        registers_->setGPR(rd, 0x80000000);
    }
    else {
        registers_->setGPR(rd, (DWORD)(dividend / divisor));
    }

    if (instruction & 0x1) {
        DWORD result = registers_->getGPR(rd);
        DWORD cr = registers_->getCR();
        cr &= ~(0xF0000000);  // Clear CR0

        if (result == 0) {
            cr |= 0x20000000;  // Set EQ bit
        }
        else if ((INT)result < 0) {
            cr |= 0x80000000;  // Set LT bit
        }
        else {
            cr |= 0x40000000;  // Set GT bit
        }

        registers_->setCR(cr);
    }
}

void PowerPCVM::executeB(DWORD instruction) {
    INT offset = (instruction & 0x03FFFFFC);
    // Sign extend if necessary
    if (offset & 0x02000000) {
        offset |= 0xFC000000;
    }

    if (instruction & 0x1) { // LK bit set
        registers_->setLR(pc_);
    }

    pc_ = pc_ + offset;
}

void PowerPCVM::executeBCOND(DWORD instruction) {
    DWORD bo = (instruction >> 21) & 0x1F;
    DWORD bi = (instruction >> 16) & 0x1F;
    short bd = instruction & 0xFFFC;

    bool condition_met = false;
    DWORD cr = registers_->getCR();
    bool cr_bit = (cr >> (31 - bi)) & 1;

    // Simplified condition checking
    if ((bo & 0x14) == 0x14) {
        condition_met = true;
    }
    else if ((bo & 0x14) == 0x4) {
        condition_met = cr_bit;
    }
    else if ((bo & 0x14) == 0x10) {
        condition_met = !cr_bit;
    }

    if (condition_met) {
        if (instruction & 0x1) { // LK bit
            registers_->setLR(pc_);
        }
        pc_ += bd;
    }
}

Memory::Memory(DWORD size) : data_(size) {}

Memory::~Memory() {
    data_.clear();
}

bool Memory::read32(DWORD address, DWORD& value) {
    if (!isValidAddress(address) || !isValidAddress(address + 3)) {
        return false;
    }

    value = (static_cast<DWORD>(data_[address]) << 24) |
        (static_cast<DWORD>(data_[address + 1]) << 16) |
        (static_cast<DWORD>(data_[address + 2]) << 8) |
        static_cast<DWORD>(data_[address + 3]);
    return true;
}

bool Memory::write32(DWORD address, DWORD value) {
    if (!isValidAddress(address) || !isValidAddress(address + 3)) {
        return false;
    }

    data_[address] = (value >> 24) & 0xFF;
    data_[address + 1] = (value >> 16) & 0xFF;
    data_[address + 2] = (value >> 8) & 0xFF;
    data_[address + 3] = value & 0xFF;
    return true;
}

bool Memory::isValidAddress(DWORD address) const {
    return address < data_.size();
}

Registers::Registers() {
    reset();
}

void Registers::reset() {
    memset(gprs_, 0, sizeof(gprs_));
    cr_ = 0;
    lr_ = 0;
}

DWORD Registers::getGPR(DWORD index) const {
    assert(index < NUM_GPRS);
    return gprs_[index];
}

void Registers::setGPR(DWORD index, DWORD value) {
    assert(index < NUM_GPRS);
    gprs_[index] = value;
}