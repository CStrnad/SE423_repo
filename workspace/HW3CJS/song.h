#define FUDGEFACTORNOTE 1
#define C4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/261.63))        //47777
#define D4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/293.66))        //42566
#define E4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/329.63))        //
#define F4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/349.23))        //
#define G4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/392.00))        //
#define A4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/440.00))        //
#define B4NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/493.88))        //
#define C5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/523.25))        //
#define D5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/587.33))        //
#define E5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/659.25))        //
#define F5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/698.46))        //
#define G5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/783.99))        //
#define A5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/880.00))        //
#define B5NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/987.77))        //
#define F4SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/369.99))   //
#define G4SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/415.3))    //
#define A4FLATNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/415.3))     //
#define C5SHARPNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/554.37))   //
#define A5FLATNOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/830.61))    //

//CJS Added a few more frequencies.
//#define G3NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/196.00))
//#define E3NOTE ((uint16_t)(((FUDGEFACTORNOTE*50000000/2)/2)/165.00))


#define OFFNOTE 1

#define SONG_LENGTH 66
uint16_t songarray[SONG_LENGTH] = {
                                    E5NOTE, E5NOTE, B4NOTE, C5NOTE,
                                    D5NOTE, D5NOTE, C5NOTE, B4NOTE,
                                    A4NOTE, A4NOTE, A4NOTE, C5NOTE,
                                    E5NOTE, E5NOTE, D5NOTE, C5NOTE,
                                    B4NOTE, B4NOTE, B4NOTE, C5NOTE,
                                    D5NOTE, D5NOTE, E5NOTE, E5NOTE,
                                    C5NOTE, C5NOTE, A4NOTE, A4NOTE, A4NOTE,
                                    A4NOTE, OFFNOTE, OFFNOTE, D5NOTE,
                                    D5NOTE, D5NOTE, F5NOTE, A5NOTE,
                                    A5NOTE, G5NOTE, F5NOTE, E5NOTE,
                                    E5NOTE, E5NOTE, C5NOTE, E5NOTE,
                                    E5NOTE, D5NOTE, C5NOTE, B4NOTE,
                                    B4NOTE, B4NOTE, C5NOTE, D5NOTE,
                                    D5NOTE, E5NOTE, E5NOTE, C5NOTE,
                                    C5NOTE, A4NOTE, A4NOTE, A4NOTE,
                                    A4NOTE, OFFNOTE, OFFNOTE
};

//
//#define SONG_LENGTH 516
//uint16_t songarray[SONG_LENGTH] = {
//A4NOTE,
//A4NOTE,
//G4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//E4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//G4NOTE,
//A4NOTE,
//A4NOTE,
//B4NOTE,
//B4NOTE,
//C5NOTE,
//C5NOTE,
//A4NOTE,
//A4NOTE,
//G4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//E4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//A4NOTE,
//A4NOTE,
//F4NOTE,
//F4NOTE,
//E4NOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//D4NOTE,
//F4NOTE,
//F4NOTE,
//A4NOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//C5NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//E4NOTE,
//E4NOTE,
//F4NOTE,
//E4NOTE,
//E4NOTE,
//G4SHARPNOTE,
//A4NOTE,
//A4NOTE,
//G4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//E4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//G4NOTE,
//A4NOTE,
//A4NOTE,
//B4NOTE,
//B4NOTE,
//C5NOTE,
//C5NOTE,
//A4NOTE,
//A4NOTE,
//G4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//E4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//A4NOTE,
//A4NOTE,
//F4NOTE,
//F4NOTE,
//E4NOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//D4NOTE,
//F4NOTE,
//F4NOTE,
//A4NOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//C5NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//E4NOTE,
//E4NOTE,
//F4NOTE,
//E4NOTE,
//E4NOTE,
//G4SHARPNOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5FLATNOTE,
//A5FLATNOTE,
//E5NOTE,
//F5NOTE,
//F5NOTE,
//D5NOTE,
//E5NOTE,
//E5NOTE,
//C5NOTE,
//D5NOTE,
//D5NOTE,
//B4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5FLATNOTE,
//A5FLATNOTE,
//E5NOTE,
//F5NOTE,
//F5NOTE,
//A5NOTE,
//A5FLATNOTE,
//A5FLATNOTE,
//E5NOTE,
//D5NOTE,
//C5NOTE,
//B4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//C5NOTE,
//C5NOTE,
//C5NOTE,
//C5NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//F4NOTE,
//F4NOTE,
//F4NOTE,
//F4NOTE,
//F4NOTE,
//F4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//C5NOTE,
//C5NOTE,
//C5NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4FLATNOTE,
//A4FLATNOTE,
//A4FLATNOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//A4FLATNOTE,
//A4FLATNOTE,
//A4FLATNOTE,
//OFFNOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//A4FLATNOTE,
//A4FLATNOTE,
//A4FLATNOTE,
//OFFNOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4FLATNOTE,
//A4FLATNOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4FLATNOTE,
//A4FLATNOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//E5NOTE,
//E5NOTE,
//E5NOTE,
//E5NOTE,
//E5NOTE,
//E5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//A5NOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//E4NOTE,
//OFFNOTE,
//E4NOTE,
//OFFNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//E4NOTE,
//OFFNOTE,
//E4NOTE,
//OFFNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//E4NOTE,
//OFFNOTE,
//E4NOTE,
//OFFNOTE,
//E5NOTE,
//E5NOTE,
//E5NOTE,
//E5NOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//D5NOTE,
//OFFNOTE,
//D5NOTE,
//OFFNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//E4NOTE,
//OFFNOTE,
//E4NOTE,
//OFFNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//E4NOTE,
//OFFNOTE,
//E4NOTE,
//OFFNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//E4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//E4NOTE,
//OFFNOTE,
//E4NOTE,
//OFFNOTE,
//E5NOTE,
//E5NOTE,
//E5NOTE,
//E5NOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//G4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//F4SHARPNOTE,
//D5NOTE,
//OFFNOTE,
//D5NOTE,
//OFFNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//C5SHARPNOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//B4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//A4NOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//OFFNOTE,
//};
