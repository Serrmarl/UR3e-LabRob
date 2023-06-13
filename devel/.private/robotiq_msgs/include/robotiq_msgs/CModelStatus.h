// Generated by gencpp from file robotiq_msgs/CModelStatus.msg
// DO NOT EDIT!


#ifndef ROBOTIQ_MSGS_MESSAGE_CMODELSTATUS_H
#define ROBOTIQ_MSGS_MESSAGE_CMODELSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotiq_msgs
{
template <class ContainerAllocator>
struct CModelStatus_
{
  typedef CModelStatus_<ContainerAllocator> Type;

  CModelStatus_()
    : gACT(0)
    , gGTO(0)
    , gSTA(0)
    , gOBJ(0)
    , gFLT(0)
    , gPR(0)
    , gPO(0)
    , gCU(0)  {
    }
  CModelStatus_(const ContainerAllocator& _alloc)
    : gACT(0)
    , gGTO(0)
    , gSTA(0)
    , gOBJ(0)
    , gFLT(0)
    , gPR(0)
    , gPO(0)
    , gCU(0)  {
  (void)_alloc;
    }



   typedef uint8_t _gACT_type;
  _gACT_type gACT;

   typedef uint8_t _gGTO_type;
  _gGTO_type gGTO;

   typedef uint8_t _gSTA_type;
  _gSTA_type gSTA;

   typedef uint8_t _gOBJ_type;
  _gOBJ_type gOBJ;

   typedef uint8_t _gFLT_type;
  _gFLT_type gFLT;

   typedef uint8_t _gPR_type;
  _gPR_type gPR;

   typedef uint8_t _gPO_type;
  _gPO_type gPO;

   typedef uint8_t _gCU_type;
  _gCU_type gCU;





  typedef boost::shared_ptr< ::robotiq_msgs::CModelStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotiq_msgs::CModelStatus_<ContainerAllocator> const> ConstPtr;

}; // struct CModelStatus_

typedef ::robotiq_msgs::CModelStatus_<std::allocator<void> > CModelStatus;

typedef boost::shared_ptr< ::robotiq_msgs::CModelStatus > CModelStatusPtr;
typedef boost::shared_ptr< ::robotiq_msgs::CModelStatus const> CModelStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotiq_msgs::CModelStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotiq_msgs::CModelStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotiq_msgs::CModelStatus_<ContainerAllocator1> & lhs, const ::robotiq_msgs::CModelStatus_<ContainerAllocator2> & rhs)
{
  return lhs.gACT == rhs.gACT &&
    lhs.gGTO == rhs.gGTO &&
    lhs.gSTA == rhs.gSTA &&
    lhs.gOBJ == rhs.gOBJ &&
    lhs.gFLT == rhs.gFLT &&
    lhs.gPR == rhs.gPR &&
    lhs.gPO == rhs.gPO &&
    lhs.gCU == rhs.gCU;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotiq_msgs::CModelStatus_<ContainerAllocator1> & lhs, const ::robotiq_msgs::CModelStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotiq_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::robotiq_msgs::CModelStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotiq_msgs::CModelStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotiq_msgs::CModelStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotiq_msgs::CModelStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotiq_msgs::CModelStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotiq_msgs::CModelStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotiq_msgs::CModelStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "17d49e32c00f4f2fb4fe664060553362";
  }

  static const char* value(const ::robotiq_msgs::CModelStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x17d49e32c00f4f2fULL;
  static const uint64_t static_value2 = 0xb4fe664060553362ULL;
};

template<class ContainerAllocator>
struct DataType< ::robotiq_msgs::CModelStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotiq_msgs/CModelStatus";
  }

  static const char* value(const ::robotiq_msgs::CModelStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotiq_msgs::CModelStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 gACT \n"
"uint8 gGTO \n"
"uint8 gSTA \n"
"uint8 gOBJ \n"
"uint8 gFLT\n"
"uint8 gPR\n"
"uint8 gPO\n"
"uint8 gCU\n"
;
  }

  static const char* value(const ::robotiq_msgs::CModelStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotiq_msgs::CModelStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gACT);
      stream.next(m.gGTO);
      stream.next(m.gSTA);
      stream.next(m.gOBJ);
      stream.next(m.gFLT);
      stream.next(m.gPR);
      stream.next(m.gPO);
      stream.next(m.gCU);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CModelStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotiq_msgs::CModelStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotiq_msgs::CModelStatus_<ContainerAllocator>& v)
  {
    s << indent << "gACT: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gACT);
    s << indent << "gGTO: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gGTO);
    s << indent << "gSTA: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gSTA);
    s << indent << "gOBJ: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gOBJ);
    s << indent << "gFLT: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gFLT);
    s << indent << "gPR: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gPR);
    s << indent << "gPO: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gPO);
    s << indent << "gCU: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gCU);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTIQ_MSGS_MESSAGE_CMODELSTATUS_H