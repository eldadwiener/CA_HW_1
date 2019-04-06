/* 046267 Computer Architecture - Spring 2019 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <assert.h>
#include <math.h>

enum mode{GLOBAL,LOCAL};
enum branch{N,T};
enum state{SNT,WNT,WT,ST};

using namespace std;

class History
{
public:
    History(uint8_t nLines,unsigned int histSize, mode hmode) : _hmode(hmode), _nLines(nLines)
    {
        if (hmode == GLOBAL) nLines = 1;
        _arr = new uint8_t[nLines];
        for (int i = 0; i < nLines; ++i)
            _arr[i] = 0;
        _mask = (1 << histSize) - 1; // logical 'and' with mask to get only regLen bits
    }
    
    ~History() { delete[] _arr; }

    uint8_t getHistory(unsigned int rowNum)
    {
        // DEBUG, make sure no SEG fault
        assert(rowNum < _nLines);
        if (_hmode == GLOBAL)
            return _arr[0];
        return _arr[rowNum];
    }

    void updateHistory(unsigned int rowNum, branch res)
    {
        // DEBUG, make sure no SEG fault
        assert(rowNum < _nLines);
        if (_hmode == GLOBAL)
            rowNum = 0;
        // get rid of the MSB to make sure we don't shift too much
        _arr[rowNum] &= (_mask >> 1);
        // add new bit, 1/0 depending on res
        _arr[rowNum] = (_arr[rowNum] << 1) + res;
    }
    void resetEntry(unsigned rowNum)
    {
        // DEBUG, make sure no SEG fault
        assert(rowNum < _nLines);
        if (_hmode == GLOBAL)
            return;
        _arr[rowNum] = 0;
    }
private:
    uint8_t* _arr;
    uint8_t _mask, _nLines;
    const mode _hmode;
};

class FSMEntry
{
public:
    FSMEntry() : _fsms(NULL) {}
    ~FSMEntry() 
    { 
        if (_fsms != NULL) delete[] _fsms; 
    }

    void initFSMEntry(uint8_t nLines, state initState)
    {
        _initState = initState;
        _nLines = nLines;
        _fsms = new state[nLines];
        for (int i = 0; i < nLines; ++i)
            _fsms[i] = initState;
    }

    branch getPredict(uint8_t nLine)
    {
        // DEBUG, make sure we don't overflow
        assert(nLine < _nLines);
        if (_fsms[nLine] == WT || _fsms[nLine] == ST)
            return T;
        return N;
    }

    void updateFSM(uint8_t nLine, branch res)
    {
        // DEBUG, make sure we don't overflow
        assert(nLine < _nLines);
        switch (_fsms[nLine]) 
        {
        case SNT:
            _fsms[nLine] = ((res == T) ? WNT : SNT);
            break;
        case WNT:
            _fsms[nLine] = ((res == T) ? WT : SNT);
            break;
        case WT:
            _fsms[nLine] = ((res == T) ? ST : WNT);
            break;
        case ST:
            _fsms[nLine] = ((res == T) ? ST : WT);
            break;
        }
    }
    void resetEntry()
    {
        for (int i = 0; i < _nLines; ++i)
            _fsms[i] = _initState;
    }
private:
    state* _fsms;
    uint8_t _nLines;
    state _initState;
};

class FSM
{
public:
    FSM(uint32_t nLines, mode fmode, unsigned int histSize, state initState) : _nLines(nLines), _fmode(fmode), _histSize(histSize)
    {
        if (fmode == GLOBAL) nLines = 1;
        _entries = new FSMEntry[nLines];
        for(int i=0; i< nLines; ++i)
            _entries[i].initFSMEntry(pow(2,histSize),initState);
    }
    ~FSM() { delete[] _entries; }

    branch getPredict(uint32_t rowNum, uint8_t history)
    {
        if (_fmode == GLOBAL) // theres only 1 FSM
            rowNum = 0;
        return _entries[rowNum].getPredict(history);
    }

    void updatePredict(uint32_t rowNum, uint8_t history, branch res)
    {
        if (_fmode == GLOBAL) // theres only 1 FSM
            rowNum = 0;
        _entries[rowNum].updateFSM(history,res);
    }

    void resetEntry(uint32_t rowNum)
    {
        // DEBUG, make sure no SEG fault
        assert(rowNum < _nLines);
        if (_fmode == GLOBAL)
            return;
        _entries[rowNum].resetEntry();
    }
private:
    mode _fmode;
    FSMEntry* _entries;
    unsigned int _histSize;
    uint32_t _nLines;
};

class BTBEntry
{
public:
    BTBEntry() : tag(0), targ(0) {}
    void updateBTBEntry(uint32_t _tag, uint32_t _targ)
    {
        tag = _tag;
        targ = _targ;
    }
    uint32_t tag, targ;
};

class BTB
{
public:
    BTB(unsigned int numEntries, unsigned int histSize, unsigned int tagSize, state fsmInitState, mode hmode, mode fmode, int shared):
            _history(numEntries,histSize,hmode),
            _fsm(numEntries,fmode,histSize,fsmInitState),
            _BTBentries(), _numEntries(numEntries), _tagMask((1 << tagSize) - 1), _rowMask((1 << numEntries) - 1) {}

    bool predict(uint32_t pc, uint32_t *dst);
    void update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);
    void GetStats(SIM_stats *curStats);


private:
    History _history;
    FSM _fsm;
    BTBEntry _BTBentries;
    unsigned int _numEntries, _tagMask, _rowMask;
    // TODO what to do with 'using' parameters
};

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	return -1;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	return;
}

void BP_GetStats(SIM_stats *curStats){
	return;
}

