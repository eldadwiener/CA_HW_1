/* 046267 Computer Architecture - Spring 2019 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <math.h>
#include <cstddef>
#include <bitset>
#include <iostream>
#define NDEBUG
#include <assert.h>

enum mode{GLOBAL,LOCAL};
enum branch{N,T};
enum state{SNT,WNT,WT,ST};
enum shared{NUN = 0,LSB = 2,MID = 16}; // shift pc by shared and 'and' with history mask, to get the needed bits for the XOR

using namespace std;

class BTB;
BTB* btb = NULL;


// History Class:
//   Used to manage the history registers for the entire BTB.
//   In charge of all history decisions, the BTB just asks to get/update the history
//   based on the row number in the BTB, and handles global history as well.
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
    
    // get/update methods: 
    //      if we are in GLOBAL mode, treat the array as a normal pointer instead.
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
        _arr[rowNum] &= (_mask >> 1);
        _arr[rowNum] = (_arr[rowNum] << 1) + res;
    }

    void resetEntry(unsigned rowNum)
    {
        // DEBUG, make sure no SEG fault
        assert(rowNum < _nLines);
        if (_hmode == GLOBAL) // reset is not done if we are in global mode.
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

    // since we are dynamically allocating the FSM entry array,
    // we use a default constructor and do the init manually.
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
        // return T if we are in Strongly-Taken or Weakly-Taken states
		if (_fsms[nLine] == WT || _fsms[nLine] == ST)
        {
			return T;
		}
        return N;
    }

    void updateFSM(uint8_t nLine, branch res)
    {
        // DEBUG, make sure we don't overflow
        assert(nLine < _nLines);
        // decide next state based on current state + new result
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


// FSM Class:
//   Used to manage the FSM's for the entire BTB.
//   Will keep track of all FSM's, and execute requests made by the BTB.
//   Just like with the history class, the fsm will handle all special cases itself.
class FSM
{
public:
    FSM(uint32_t nLines, mode fmode, unsigned int histSize, state initState) : _fsmMode(fmode), _histSize(histSize), _nLines(nLines) 
    {
        if (fmode == GLOBAL) nLines = 1;
        _entries = new FSMEntry[nLines];
        for(unsigned i=0; i< nLines; ++i)
            _entries[i].initFSMEntry(pow(2,histSize),initState);
    }
    ~FSM() { delete[] _entries; }

    // go to the relevant FSM entry (indicated by rowNum), 
    // or to the global one if we are in global mode,
    // and request a prediction for the given history
    branch getPredict(uint32_t rowNum, uint8_t history)
    {
        if (_fsmMode == GLOBAL) // theres only 1 FSM
            rowNum = 0;
        return _entries[rowNum].getPredict(history);
    }

    void updatePredict(uint32_t rowNum, uint8_t history, branch res)
    {
        if (_fsmMode == GLOBAL) // theres only 1 FSM
            rowNum = 0;
        _entries[rowNum].updateFSM(history,res);
    }

    void resetEntry(uint32_t rowNum)
    {
        // DEBUG, make sure no SEG fault
        assert(rowNum < _nLines);
        if (_fsmMode == GLOBAL)
            return;
        _entries[rowNum].resetEntry();
    }
private:
    mode _fsmMode;
    FSMEntry* _entries;
    unsigned int _histSize;
    uint32_t _nLines;
};

// BTBEntry Class:
//   each entry corresponds to a row number in the BTB,
//   and keeps track of the row's current tag and predicted destination address.
//   also keeps a valid bit to make sure we look at valid data.
class BTBEntry
{
public:
    BTBEntry() : tag(0), targ(0), valid(false) {}
    void updateBTBEntry(uint32_t _tag, uint32_t _targ)
    {
        tag = _tag;
        targ = _targ;
    }
    uint32_t tag, targ;
	bool valid;
};

// BTBEntry Class:
//   Represents the BTB in the system, and is the main entity operating in the code.
//   the BTB keeps the history,fsm and entries of the system.
//   it parses the commands given by the main functions (BP_init/predict/update/ GetStats)
//   and directs the relevant instructions to the history and fsm objects.
class BTB
{
public:
    // pretty much initiates the whole system, very wow
    BTB(unsigned int numEntries, unsigned int histSize, unsigned int tagSize, state fsmInitState, mode hmode, mode fmode, int shared):
            _history(numEntries,histSize,hmode), _fsm(numEntries,fmode,histSize,fsmInitState),
            _numEntries(numEntries), _tagMask((1 << tagSize) - 1), _rowMask(numEntries - 1), _histMask((1 << histSize) - 1),            
            _brNum(0), _flushNum(0)
    {
		_size = numEntries * (tagSize + 30) //BTB size = (num of entries)*(size of line) TODO:maybe 30 instead of 32 
			+ histSize * (numEntries*hmode + 1 - hmode) //hist size = histsize*(num of hists)
			+ 2*pow(2,histSize)*(numEntries*fmode + 1 - fmode);// FSM size = (num of FSMs)*(sizeof FSM entry=2)*2^(histsize)
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

    ~BTB() { delete[] _BTBentries; }

    
    bool predict(uint32_t pc, uint32_t *dst)
    {
        // Get row num, and Tag
        unsigned row = ((pc >> 2) & _rowMask); //remove 2 lsb zeroes, and taken same amount of bits as in the mask.
        unsigned tag = ((pc >> 2) & _tagMask);
        // Check if such a tag exists in the BTB
        if (_BTBentries[row].tag != tag || _BTBentries[row].valid == false) // no prediction available
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
        // update stats
        ++_brNum;
        if ((taken && (pred_dst != targetPc))
            || (!taken && (pred_dst != pc + 4)))
        {
            ++_flushNum;
        }
        // Get row num, and Tag
        // remove 2 lsb zeroes, and taken same amount of bits as in the mask.
        unsigned row = ((pc >> 2) & _rowMask); 
        unsigned tag = ((pc >> 2) & _tagMask);
        // Check if such a tag exists in the BTB
        if (_BTBentries[row].tag != tag ) // Tag does not exist 
        {   
            // (these will check if they are in global mode by themselves)
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
		_BTBentries[row].valid = true;
    }

    void GetStats(SIM_stats *curStats)
    {   
        curStats->br_num = _brNum;
        curStats->flush_num = _flushNum;
        curStats->size = _size;
    }


private:
    History _history;
    FSM _fsm;
    BTBEntry* _BTBentries;
    unsigned int _numEntries, _tagMask, _rowMask, _histMask;
    shared _shareMode;
    unsigned int _brNum, _flushNum, _size;
};

// parse the inputs and create the BTB, WOOHOO
int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
    if (btb != NULL) // already initiated
	    return -1;
    mode hmode = (isGlobalHist == true) ? GLOBAL : LOCAL;
    mode fmode = (isGlobalTable == true) ? GLOBAL : LOCAL;
    state initstate = static_cast<state>(fsmState);
    btb = new BTB(btbSize, historySize, tagSize, initstate, hmode, fmode, Shared);
    return 0;
}

// Simply let the BTB handle the rest of these, he is a god 
bool BP_predict(uint32_t pc, uint32_t *dst){
	return btb->predict(pc,dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
    btb->update(pc, targetPc, taken, pred_dst);
	return;
}

void BP_GetStats(SIM_stats *curStats){
    btb->GetStats(curStats);
	delete btb;
	return;
}


