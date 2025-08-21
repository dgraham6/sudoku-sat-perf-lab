#include "stdio.h"
#include <cstdint>

namespace {

int InBlock[81], InRow[81], InCol[81];

const int BLANK = 0;
const int ONES = 0x3fe;   // Binary 1111111110

int Entry[81];  // Records entries 1-9 in the grid, as the corresponding bit set to 1
int Block[9], Row[9], Col[9]; // Each int is a 9-bit array

int SeqPtr = 0;
int Sequence[81];

int Limit = 0;
int SolutionCount = 0;
int GuessCount = 0;
int LevelCount[81];


void SwapSeqEntries(int S1, int S2)
{
    int temp = Sequence[S2];
    Sequence[S2] = Sequence[S1];
    Sequence[S1] = temp;
}


bool InitEntry(int i, int j, int val)
{
    int Square = 9 * i + j;
    int valbit = 1 << val;
    int SeqPtr2;

    if (Block[InBlock[Square]] & valbit && Col[InCol[Square]] & valbit && Row[InRow[Square]] & valbit) {
        Entry[Square] = valbit;
        Block[InBlock[Square]] &= ~valbit;
        Col[InCol[Square]] &= ~valbit; // Simpler Col[j] &= ~valbit;
        Row[InRow[Square]] &= ~valbit; // Simpler Row[i] &= ~valbit;
    } else {
        return false;
    }

    SeqPtr2 = SeqPtr;
    while (SeqPtr2 < 81 && Sequence[SeqPtr2] != Square)
        SeqPtr2++ ;

    SwapSeqEntries(SeqPtr, SeqPtr2);
    SeqPtr++;
    return true;
}


bool InitPuzzle(const char *puzzle) {
    int i, j;

    for (i = 0; i < 9; i++) {
        for (j = 0; j < 9; j++) {
            char ch = puzzle[i * 9 + j];
            if (ch >= '1' && ch <= '9')
                if (!InitEntry(i, j, ch - '0')) return false;
        }
    }
    return true;
}


int NextSeq(int S) 
{
    int S2, Square, Possibles, BitCount;
    int T, MinBitCount = 100;

    for (T = S; T < 81; T++) {
        Square = Sequence[T];
        Possibles = Block[InBlock[Square]] & Row[InRow[Square]] & Col[InCol[Square]];
        BitCount = 0;
        while (Possibles) {
           Possibles &= ~(Possibles & -Possibles);
           BitCount++;
        }

        if (BitCount < MinBitCount) {
           MinBitCount = BitCount;
           S2 = T;
        }
    }

    return S2;
}


void Place(int S)
{
    LevelCount[S]++;

    if (S >= 81) {
        SolutionCount++;
        return;
    }

    int S2 = NextSeq(S);
    SwapSeqEntries(S, S2);

    int Square = Sequence[S];

    int BlockIndex = InBlock[Square],
        RowIndex = InRow[Square],
        ColIndex = InCol[Square];

    int Possibles = Block[BlockIndex] & Row[RowIndex] & Col[ColIndex];
    while (Possibles) {
        int valbit = Possibles & (-Possibles); // Lowest 1 bit in Possibles
        Possibles &= ~valbit;
        Entry[Square] = valbit;
        Block[BlockIndex] &= ~valbit;
        Row[RowIndex] &= ~valbit;
        Col[ColIndex] &= ~valbit;

        GuessCount += (Possibles == 0 ? 0 : 1);
        Place(S + 1);
        if (SolutionCount == Limit) return;

        Entry[Square] = BLANK; // Could be moved out of the loop
        Block[BlockIndex] |= valbit;
        Row[RowIndex] |= valbit;
        Col[ColIndex] |= valbit;
    }

    SwapSeqEntries(S, S2);
}


static bool Initialized = false;


void InitTables() {
    int i, j, Square;

    for (i = 0; i < 9; i++)
        for (j = 0; j < 9; j++) {
            Square = 9 * i + j;
            InRow[Square] = i;
            InCol[Square] = j;
            InBlock[Square] = (i / 3) * 3 + ( j / 3);
        }


    for (Square = 0; Square < 81; Square++) {
        Sequence[Square] = Square;
        Entry[Square] = BLANK;
        LevelCount[Square] = 0;
    }

    Initialized = true;
}

} // namespace


extern "C"
size_t OtherSolverLHLSudoku(const char *input,
                            size_t limit, uint32_t unused_configuration,
                            char *solution, size_t *num_guesses)
{
    if (!Initialized) InitTables();

    for (int i = 0; i < 9; i++)
        Block[i] = Row[i] = Col[i] = ONES;

    Limit = limit;
    SeqPtr = 0;
    SolutionCount = 0;
    GuessCount = 0;

    if (InitPuzzle(input)) {
        Place(SeqPtr);
        *num_guesses = GuessCount;
        return SolutionCount;
    } else {
        *num_guesses = 0;
        return 0;
    }
}
