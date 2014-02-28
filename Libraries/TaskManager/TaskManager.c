//*****************************************************************************
// TaskManager
// Version 2.0 Fev 2005
//
// 2.0 -> -Re work all the interrupt code
// 1.0 -> -Everything is new
//
// Sylvain Bissonnette
//*****************************************************************************
// Editor : UltraEdit32
//*****************************************************************************
//                T I M E R   U S A G E
//
// Timer 0 is use by Task Manager
//
//*****************************************************************************
//
//*****************************************************************************
//                      I N C L U D E
//*****************************************************************************
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <STRING.H>
#include "TaskManager.h"
//#define MINIMUM_CODE    // if def save 71 words of code but no more
                          // checking of Register and UnRegister

//*****************************************************************************
//            G L O B A L   V A R I A B L E
//*****************************************************************************
typedef struct Task
{
  FuncPTR Function;
  uint16_t Parameter;
  uint16_t Interval;
  uint16_t Ticker;
  uint8_t Persiste;
}Task;
Task TaskList[MAX_TASK];
Task TaskAdd;
Task TaskDel;
int TaskMax = 0;

/******************************************************************************

Name:         void TaskInit(void)

Description:  Init task system

Input:        none

Output:       none

Misc:         Use Timer 0

******************************************************************************/
void TaskInit(void)
{
	uint8_t i;

  for (i=0;i<TaskMax;i++)
  {
    TaskList[i].Function = NULL;
    TaskList[i].Parameter = 0xffff;
    TaskList[i].Interval = 0xffff;
    TaskList[i].Persiste = 0xff;
  }

  //Timer0
  TCCR0 = 0x02;               // Timer0 / 8
  TIMSK |= (1<<TOIE0);        // int enable on Timer 0 overflow
}

/******************************************************************************

Name:         int TaskRegister( void(*FunctionPTR)(void),
                                int Interval,
                                uint8_t Persiste)

Description:  Register a function to be call

Input:        void  Function pointer
              int   Interval
              uchar Persiste

Output:       0 -> Task not registrated (error)
              1 -> Task is now registrated

Misc:

******************************************************************************/
int TaskRegister(   FuncPTR Function,
                    int Parameter,
                    uint16_t Interval,
                    uint8_t Persiste)
{
  uint16_t i = 0;

  if (Function == NULL) return 0;
  if (TaskMax >= MAX_TASK) return 0;

  while (TaskAdd.Function != NULL)
  {
    wdt_reset();
    if (i++ > 65530) return 0;
  }

  TaskAdd.Function = Function;
  TaskAdd.Parameter = Parameter;
  TaskAdd.Interval = Interval;
  TaskAdd.Persiste = Persiste;
  return 1;
}

/******************************************************************************

Name:         int TaskUnRegister(FuncPTR Function)

Description:  UnRegister a function

Input:        Function pointer

Output:       0 -> Task not find
              1 -> Task is unregistrated

Misc:

******************************************************************************/
int TaskUnRegister(FuncPTR Function)
{
  uint16_t i = 0;

  if (Function == NULL) return 0;

  while(TaskDel.Function != NULL)
  {
    wdt_reset();
    if (i++ > 65530) return 0;
  }

  TaskDel.Function = Function;
  return 1;
}

/**********************************************************

Name:         int TaskCheckRegister(void(*FunctionPTR)(void))

Description:  Check if a function is register

Input:        Function pointer

Output:       0 -> Not register
              1 -> Register

Misc:

**********************************************************/
int TaskCheckRegister(FuncPTR Function)
{
  uint8_t i;

  for (i=0;i<TaskMax;i++)
  {
    if (TaskList[i].Function == Function) return 1;
  }
  return 0;
}

/**********************************************************

Name:         int TaskCheckRegisterWParameter(void(*FunctionPTR)(void),int)

Description:  Check if a function is register with is parameter

Input:        Function pointer

Output:       0 -> Not register
              1 -> Register

Misc:

**********************************************************/
int TaskCheckRegisterWParameter(int Parameter)
{
  uint8_t i;

  for (i=0;i<TaskMax;i++)
  {
    if (TaskList[i].Parameter == Parameter) return 1;
  }
  return 0;
}

/******************************************************************************

Name:         int TaskUnRegisterWParameter(int Parameter)

Description:  UnRegister a function

Input:        Function pointer

Output:       0 -> Task not find
              1 -> Task is unregistrated

Misc:

******************************************************************************/
int TaskUnRegisterWParameter(int Parameter)
{
  uint16_t i = 0;

  while(TaskDel.Function != NULL)
  {
    wdt_reset();
    if (i++ > 65530) return 0;
  }

  for (i=0;i<TaskMax;i++)
  {
    if (TaskList[i].Parameter == Parameter)
    {
    	TaskDel.Function = TaskList[i].Function;
    }
  }
  return 0;
}


/**********************************************************

Name:         int TaskChangeInterval(void(*FunctionPTR)(void), uint16_t Interval)

Description:  Change the interval of a task

Input:        Function pointer
							Interval

Output:       0 -> Not register
              1 -> Register

Misc:

**********************************************************/
int TaskChangeInterval(FuncPTR Function, uint16_t Interval)
{
  uint8_t i;

  for (i=0;i<TaskMax;i++)
  {
    if (TaskList[i].Function == Function)
    {
    	TaskList[i].Interval = Interval;
    	return 1;
    }
  }
	return 0;
}

/**********************************************************

Name:         void TaskStop(void)

Description:  Stop Task Execution

Input:        none

Output:       none

Misc:

**********************************************************/
void TaskStop(void)
{
  TIMSK &= ~(1<<TOIE0);   // int disable on Timer 0 overflow
}

/**********************************************************

Name:         void TaskStart(void)

Description:  Start Task Execution

Input:        none

Output:       none

Misc:

**********************************************************/
void TaskStart(void)
{
  TIMSK |= (1<<TOIE0);    // int enable on Timer 0 overflow
}

/**********************************************************

Name:         void TaskExecute(void)

Description:  TaskExecute

Input:        none

Output:       none

Misc:         TaskExecute is execute each 100us

**********************************************************/
//#pragma interrupt_handler TaskExecute:12
ISR(TIMER0_OVF_vect)
{
  static uint8_t i,j;
  static FuncPTR Function;

  TCNT0 = 255 - (F_CPU / 8UL / 10000UL);
  TaskStop();
  wdt_reset();

  if (TaskDel.Function != NULL) _TaskUnRegister();
  if (TaskAdd.Function != NULL) _TaskRegister();

  for (i=0;i<TaskMax;i++)
  {
    if (TaskList[i].Ticker++ >= TaskList[i].Interval)
    {
      Function = TaskList[i].Function;
      if (!TaskList[i].Persiste)
      {
        for (j=i;j<TaskMax;j++) memcpy(&TaskList[j],&TaskList[j+1],sizeof(Task));
        TaskMax--;
      }
      else
      {
        TaskList[i].Ticker = 0;
      }
      sei();
      Function(TaskList[i].Parameter);
      cli();
    }
  }
  TaskStart();
}

/******************************************************************************

Name:         void _TaskUnRegister(void)

Description:  UnRegister a function

Input:        TaskDel struct

Output:       none

Misc:

******************************************************************************/
void _TaskUnRegister(void)
{
  uint8_t i,j;

  for (i=0;i<TaskMax;i++)
  {
    if (TaskList[i].Function == TaskDel.Function)
    {
      for (j=i;j<TaskMax;j++) memcpy(&TaskList[j],&TaskList[j+1],sizeof(Task));
      TaskMax--;
      TaskDel.Function = NULL;
      return;
    }
  }
  TaskDel.Function = NULL;
}

/******************************************************************************

Name:         int _TaskRegister(void)

Description:  Register a function

Input:        TaskAdd struct

Output:       none

Misc:

******************************************************************************/
void _TaskRegister(void)
{
  TaskList[TaskMax].Function = TaskAdd.Function;
  TaskList[TaskMax].Parameter = TaskAdd.Parameter;
  TaskList[TaskMax].Interval = TaskAdd.Interval;
  TaskList[TaskMax].Ticker = 0;
  TaskList[TaskMax].Persiste = TaskAdd.Persiste;
  TaskAdd.Function = NULL;
  TaskMax++;
}
