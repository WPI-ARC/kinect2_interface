using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;

//////////////////////////////////////////////////
/////    AUTOGENERATED MESSAGE DEFINITION    /////
//////////////////////////////////////////////////
/////         DO NOT MODIFY BY HAND!         /////
//////////////////////////////////////////////////

namespace ROS_CS
{
    namespace actionlib_msgs
    {
        public class GoalStatusArray : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "GoalStatusArray";
            public readonly string md5sum = "8b2b82f13216d0a8ea88bd3af735e619";
            public std_msgs.Header header;
            public List<actionlib_msgs.GoalStatus> status_list;

            public GoalStatusArray ()
            {
                header = new std_msgs.Header();
                status_list = new List<actionlib_msgs.GoalStatus>();
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += header.ToStringRaw() + "\n";
                string_rep += "status_list:\n[";
                foreach (actionlib_msgs.GoalStatus element in status_list)
                {
                    string_rep += " " + element.ToStringRaw();
                }
                string_rep += "]\n\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                header.Serialize(stream);
                System.Byte[] status_list_len_bytes = BitConverter.GetBytes((System.UInt32)status_list.Count);
                stream.Write(status_list_len_bytes, 0, status_list_len_bytes.Length);
                foreach(actionlib_msgs.GoalStatus element in status_list)
                {
                    element.Serialize(stream);
                }
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                curIndex += header.Deserialize(serialized, curIndex);
                System.UInt32 status_list_len = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(status_list_len).Length;
                for (int i = 0; i < (int)status_list_len; i++)
                {
                    actionlib_msgs.GoalStatus element = new actionlib_msgs.GoalStatus();
                    curIndex += element.Deserialize(serialized, curIndex);
                    status_list.Add(element);
                }
                return (curIndex - startIndex);
            }

        }
    }
}