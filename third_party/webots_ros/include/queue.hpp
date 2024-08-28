#ifndef QUEUE_H
#define QUEUE_H

#include <iostream> 
#include <vector>						
#include "types.hpp"
	
using namespace std;

#define MAXSIZE	512

typedef enum {
  RES_OK = 0,
  RES_ERROR
} STATUS;
 
typedef enum {
  RECEIVED_START,
  NO_RECEIVED_START
} RECV_FSM;
 
class Queue {
  private:
    uint32 front;
    uint32 rear;
    uint8 data[MAXSIZE];

  public:
    Queue()
    {
      front = 0;
      rear = 0;
    }

    ~Queue()
    {

    }

	uint32 LIMIT(uint32  in)
	{
 	    if(in>=MAXSIZE)
	      return MAXSIZE-1;
	    else
	      return in;
	}

    STATUS EnQueue(uint8 e)
    {
      if ((rear + 1) % MAXSIZE == front)
      {
       // return RES_ERROR;
      }
      data[rear] = e;
      rear = LIMIT((rear + 1) % MAXSIZE);
      return RES_OK;  
    }

    uint32 QueueLength()
    {
      return LIMIT((rear - front + MAXSIZE) % MAXSIZE);
    }

    uint8 GetQueue(uint32 index)
    {
      return data[(front + index) % MAXSIZE];
    }

    void HandleData(vector<uint8> &message)
    {
      uint16 frame_len=0, frame_id=0, frame_startpos = 0;
      uint8  frame_check=0, c;
      RECV_FSM frame_state = NO_RECEIVED_START;
      uint32 len = LIMIT((rear - front + MAXSIZE) % MAXSIZE);	
      for (uint32 i = 0; i < len; i++)
      {
        if(i + 6 > len) break; //6 数据帧的最小长�?

	if (frame_state == NO_RECEIVED_START)
	{
	  if (GetQueue(i) == 0xAA && GetQueue(i+1) == 0xAF)				
	  {
	  // cout<<"get frame"<<endl;
	   frame_check = 0;
             frame_state = RECEIVED_START;
	   }
	  else
            frame_startpos = i + 1;					
	} 
	else if (frame_state == RECEIVED_START)
	{
	  frame_id   = GetQueue(frame_startpos+2) ;
            frame_len  = GetQueue(frame_startpos+3) ;
	//  cout<<"frame_id: "<<frame_id<<" frame_len: "<<frame_len<<endl;
          if(frame_len > 100)
          {
             frame_state = NO_RECEIVED_START;
             i += 4;
	   frame_startpos = i;					
	   continue;
          }

	 for(uint16  j=0;j<(frame_len+4);j++)
	    frame_check+=GetQueue(frame_startpos+j) ;
	 if((frame_check==GetQueue(frame_len+4)))		
	 {           
	   // cout<<"check right"<<endl;
              for(uint16 j = 0; j < frame_len+4; j ++ )	   
                message.push_back(GetQueue(frame_startpos+j));

              frame_state = NO_RECEIVED_START;	
              frame_startpos = i + 1;				
              break;						
	  }
          else
          {
	  //cout<<"check error"<<endl;
            break;
          }
        }
      }
      front = LIMIT((front + frame_startpos) % MAXSIZE);			
    }
};

#endif
