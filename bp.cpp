/* 046267 Computer Architecture - Spring 2019 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <assert.h>
#include <math.h>
#include <cstddef>

enum mode{GLOBAL,LOCAL};
enum branch{N,T};
enum state{SNT,WNT,WT,ST};
enum shared{NUN = 0,LSB = 2,MID = 16}; // shift pc by shared and 'and' with history mask, to get the needed bits for the XOR

using namespace std;

class BTB;
BTB* btb = NULL;

class History
{
public:
    History(uint8_t nLines,unsigned int histSize, mode hmode) : _nLines(nLines), _hmode(hmode) 
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
    FSM(uint32_t nLines, mode fmode, unsigned int histSize, state initState) : _fmode(fmode), _histSize(histSize), _nLines(nLines) 
    {
        if (fmode == GLOBAL) nLines = 1;
        _entries = new FSMEntry[nLines];
        for(unsigned i=0; i< nLines; ++i)
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
            _history(numEntries,histSize,hmode), _fsm(numEntries,fmode,histSize,fsmInitState),
            _numEntries(numEntries), _tagMask((1 << tagSize) - 1), _rowMask(numEntries - 1), _histMask((1 << histSize) - 1),            
            _brNum(0), _flushNum(0)
    {
        _BTBentries = new BTBEntry[numEntries];
        if (shared == 0)
        {
            _shareMode = NUN;
            _histMask = 0; // don't need to XOR at all, so will XOR with 0
        }
        else if (shared == 1) 
            _shareMode = LSB;
        else 
            _shareMode = MID;
    }
    bool predict(uint32_t pc, uint32_t *dst)
    {
        // Get row num, and Tag
        unsigned row = ((pc >> 2) & _rowMask); //remove 2 lsb zeroes, and taken same amount of bits as in the mask.
        unsigned tag = ((pc >> 2) & _tagMask);
        // Check if such a tag exists in the BTB
        // TODO: is it possible for the PC the be 0x0? what happens if the btbentry wasn't used yet?
        if (_BTBentries[row].tag != tag) // no prediction available
        {
            *dst = pc + 4;
            return false;
        }
        // Prediction available, get history
        uint8_t hist = _history.getHistory(row);
        // fix history in case we use l/g-share
        hist = ( (pc >> _shareMode) & _histMask) ^ hist;
        // finally get prediction for this given history
        branch predict = _fsm.getPredict(row, hist);
        if (predict == N)
        {
            *dst = pc + 4;
            return false;
        }
        else
        {
            *dst = _BTBentries[row].targ;
            return true;
        }
    }
    void update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst)
    {
        //TODO: check what needs to happen if the prediction was correct, but wrong dest address,
        //      do we completely reset the BTB entry? or just update the destination?

        // update stats
        ++_brNum;
        if ((taken && (pred_dst != targetPc))
            || (!taken && (pred_dst != pc + 4)))
        {
            ++_flushNum;
        }
        // Get row num, and Tag
        unsigned row = ((pc >> 2) & _rowMask); //remove 2 lsb zeroes, and taken same amount of bits as in the mask.
        unsigned tag = ((pc >> 2) & _tagMask);
        // Check if such a tag exists in the BTB
        // TODO: is it possible for the PC the be 0x0? what happens if the btbentry wasn't used yet?
        if (_BTBentries[row].tag != tag) // Tag does not exist 
        {   // TODO what happens if only need to update the targetPC?
            // these will not reset if they are global
            _history.resetEntry(row);
            _fsm.resetEntry(row);
        }
        // update BTB entry
        _BTBentries[row].updateBTBEntry(tag, targetPc);
        // get old history to update FSM, and update history
        uint8_t hist = _history.getHistory(row);
        branch res = (taken) ? T : N;
        _history.updateHistory(row, res);
        // fix hist in case we use l/g-share, and update FSM
        hist = ( (pc >> _shareMode) & _histMask) ^ hist;
        _fsm.updatePredict(row, hist, res);
    }
    void GetStats(SIM_stats *curStats)
    {   // TODO: THIS
        curStats->br_num = _brNum;
        curStats->flush_num = _flushNum;
        curStats->size = 0;
    }


private:
    History _history;
    FSM _fsm;
    BTBEntry* _BTBentries;
    unsigned int _numEntries, _tagMask, _rowMask, _histMask;
    shared _shareMode;
    unsigned int _brNum, _flushNum;
};

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
    if (btb != NULL) // already initiated
	    return -1;
    mode hmode = (isGlobalHist == true) ? GLOBAL : LOCAL;
    mode fmode = (isGlobalTable == true) ? GLOBAL : LOCAL;
    state initstate;
    // TODO: check if you can cast fsmState into (state) directly, this shiz is ugly
    switch (fsmState)
    {
    case 0:
        initstate = SNT;
        break;
    case 1:
        initstate = WNT;
        break;
    case 2:
        initstate = WT;
        break;
    case 3:
        initstate = ST;
        break;
    }
    btb = new BTB(btbSize, historySize, tagSize, initstate, hmode, fmode, Shared);
    return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	return btb->predict(pc,dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
    btb->update(pc, targetPc, taken, pred_dst);
	return;
}

void BP_GetStats(SIM_stats *curStats){
    btb->GetStats(curStats);
	return;
}


