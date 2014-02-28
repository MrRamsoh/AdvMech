//*****************************************************************************
// TaskManager.h
// Version 1.0 Dec 2004
//
// 1.0 -> -Everything is new
//
// Sylvain Bissonnette
//*****************************************************************************
//
//*****************************************************************************
//                      D E F I N E
//*****************************************************************************
#define TASK_MAN_VER    10
#define TRUE            1
#define FALSE           0
#define XTAL            16000000

#define MAX_TASK        10

#define T100US          1
#define T200US          2
#define T300US          3
#define T400US          4
#define T500US          5
#define T600US          6
#define T700US          7
#define T800US          8
#define T900US          9

#define T1MS            10
#define T2MS            20
#define T3MS            30
#define T4MS            40
#define T5MS            50
#define T6MS            60
#define T7MS            70
#define T8MS            80
#define T9MS            90

#define T10MS           100
#define T20MS           200
#define T30MS           300
#define T40MS           400
#define T50MS           500
#define T60MS           600
#define T70MS           700
#define T80MS           800
#define T90MS           900

#define T100MS          1000
#define T110MS          1100
#define T120MS          1200
#define T130MS          1300
#define T140MS          1400
#define T150MS          1500
#define T160MS          1600
#define T170MS          1700
#define T180MS          1800
#define T190MS          1900

#define T200MS          2000
#define T210MS          2100
#define T220MS          2200
#define T230MS          2300
#define T240MS          2400
#define T250MS          2500
#define T260MS          2600
#define T270MS          2700
#define T280MS          2800
#define T290MS          2900

#define T300MS          3000
#define T310MS          3100
#define T320MS          3200
#define T330MS          3300
#define T340MS          3400
#define T350MS          3500
#define T360MS          3600
#define T370MS          3700
#define T380MS          3800
#define T390MS          3900

#define T400MS          4000
#define T410MS          4100
#define T420MS          4200
#define T430MS          4300
#define T440MS          4400
#define T450MS          4500
#define T460MS          4600
#define T470MS          4700
#define T480MS          4800
#define T490MS          4900

#define T500MS          5000
#define T510MS          5100
#define T520MS          5200
#define T530MS          5300
#define T540MS          5400
#define T550MS          5500
#define T560MS          5600
#define T570MS          5700
#define T580MS          5800
#define T590MS          5900

#define T600MS          6000
#define T610MS          6100
#define T620MS          6200
#define T630MS          6300
#define T640MS          6400
#define T650MS          6500
#define T660MS          6600
#define T670MS          6700
#define T680MS          6800
#define T690MS          6900

#define T700MS          7000
#define T710MS          7100
#define T720MS          7200
#define T730MS          7300
#define T740MS          7400
#define T750MS          7500
#define T760MS          7600
#define T770MS          7700
#define T780MS          7800
#define T790MS          7900

#define T800MS          8000
#define T810MS          8100
#define T820MS          8200
#define T830MS          8300
#define T840MS          8400
#define T850MS          8500
#define T860MS          8600
#define T870MS          8700
#define T880MS          8800
#define T890MS          8900

#define T900MS          9000
#define T910MS          9100
#define T920MS          9200
#define T930MS          9300
#define T940MS          9400
#define T950MS          9500
#define T960MS          9600
#define T970MS          9700
#define T980MS          9800
#define T990MS          9900

#define T1S             10000
#define T2S             20000
#define T3S             30000
#define T4S             40000
#define T5S             50000
#define T6S             60000
#define T7S             70000
#define T8S             80000
#define T9S             90000

#define T10S            100000
#define T11S            110000
#define T12S            120000
#define T13S            130000
#define T14S            140000
#define T15S            150000
#define T16S            160000
#define T17S            170000
#define T18S            180000
#define T19S            190000

#define T20S            200000
#define T21S            210000
#define T22S            220000
#define T23S            230000
#define T24S            240000
#define T25S            250000
#define T26S            260000
#define T27S            270000
#define T28S            280000
#define T29S            290000

#define T30S            300000
#define T31S            310000
#define T32S            320000

//*****************************************************************************
//                  P R O T O T Y P E
//*****************************************************************************
typedef void (*FuncPTR)(int);
void TaskInit(void);
int TaskRegister(FuncPTR Function,int Parameter,uint Interval,ushort Persiste);
int TaskUnRegister(FuncPTR Function);
int TaskCheckRegister(FuncPTR);
int TaskCheckRegisterWParameter(int);
int TaskUnRegisterWParameter(int);
int TaskChangeInterval(FuncPTR, uint Interval);
void TaskStop(void);
void TaskStart(void);
void TaskExecute(void);
void _TaskUnRegister(void);
void _TaskRegister(void);