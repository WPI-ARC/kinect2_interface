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
    namespace teleop_msgs
    {
        public class AggregatedMessages : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "AggregatedMessages";
            public readonly string md5sum = "cd9967faad6e96eb969348311d258f2f";
            public std_msgs.Header header;
            public List<teleop_msgs.SerializedMessage> Messages;

            public AggregatedMessages ()
            {
                header = new std_msgs.Header();
                Messages = new List<teleop_msgs.SerializedMessage>();
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += header.ToStringRaw() + "\n";
                string_rep += "Messages:\n[";
                foreach (teleop_msgs.SerializedMessage element in Messages)
                {
                    string_rep += " " + element.ToStringRaw();
                }
                string_rep += "]\n\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                header.Serialize(stream);
                System.Byte[] Messages_len_bytes = BitConverter.GetBytes((System.UInt32)Messages.Count);
                stream.Write(Messages_len_bytes, 0, Messages_len_bytes.Length);
                foreach(teleop_msgs.SerializedMessage element in Messages)
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
                System.UInt32 Messages_len = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(Messages_len).Length;
                for (int i = 0; i < (int)Messages_len; i++)
                {
                    teleop_msgs.SerializedMessage element = new teleop_msgs.SerializedMessage();
                    curIndex += element.Deserialize(serialized, curIndex);
                    Messages.Add(element);
                }
                return (curIndex - startIndex);
            }

        }
    }
}
