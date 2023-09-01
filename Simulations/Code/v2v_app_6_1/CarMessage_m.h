//
// Generated file, do not edit! Created by nedtool 5.3 from CarMessage.msg.
//

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#ifndef __CARMESSAGE_M_H
#define __CARMESSAGE_M_H

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0503
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
 
#include "veins/modules/messages/WaveShortMessage_m.h"
// }}

/**
 * Class generated from <tt>CarMessage.msg:9</tt> by nedtool.
 * <pre>
 * packet CarMessage extends WaveShortMessage
 * {
 *     string helloMsg;
 *     int priority;
 *     string rsuId;
 *     string carId;
 *     simtime_t sendTime;
 * }
 * </pre>
 */
class CarMessage : public ::WaveShortMessage
{
  protected:
    ::omnetpp::opp_string helloMsg;
    int priority;
    ::omnetpp::opp_string rsuId;
    ::omnetpp::opp_string carId;
    ::omnetpp::simtime_t sendTime;

  private:
    void copy(const CarMessage& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const CarMessage&);

  public:
    CarMessage(const char *name=nullptr, short kind=0);
    CarMessage(const CarMessage& other);
    virtual ~CarMessage();
    CarMessage& operator=(const CarMessage& other);
    virtual CarMessage *dup() const override {return new CarMessage(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual const char * getHelloMsg() const;
    virtual void setHelloMsg(const char * helloMsg);
    virtual int getPriority() const;
    virtual void setPriority(int priority);
    virtual const char * getRsuId() const;
    virtual void setRsuId(const char * rsuId);
    virtual const char * getCarId() const;
    virtual void setCarId(const char * carId);
    virtual ::omnetpp::simtime_t getSendTime() const;
    virtual void setSendTime(::omnetpp::simtime_t sendTime);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const CarMessage& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, CarMessage& obj) {obj.parsimUnpack(b);}


#endif // ifndef __CARMESSAGE_M_H

