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
    namespace std_msgs
    {
        public class UInt16MultiArray : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "UInt16MultiArray";
            public readonly string md5sum = "52f264f1c973c4b73790d384c6cb4484";
            public std_msgs.MultiArrayLayout layout;
            public List<System.UInt16> data;

            public UInt16MultiArray ()
            {
                layout = new std_msgs.MultiArrayLayout();
                data = new List<System.UInt16>();
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += layout.ToStringRaw() + "\n";
                string_rep += "data:\n[";
                foreach (System.UInt16 element in data)
                {
                    string_rep += " " + Convert.ToString(element);
                }
                string_rep += "]\n\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                layout.Serialize(stream);
                System.Byte[] data_len_bytes = BitConverter.GetBytes((System.UInt32)data.Count);
                stream.Write(data_len_bytes, 0, data_len_bytes.Length);
                foreach(System.UInt16 element in data)
                {
                    System.Byte[] element_bytes = BitConverter.GetBytes(element);
                    stream.Write(element_bytes, 0, element_bytes.Length);
                }
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                curIndex += layout.Deserialize(serialized, curIndex);
                System.UInt32 data_len = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(data_len).Length;
                for (int i = 0; i < (int)data_len; i++)
                {
                    System.UInt16 element = BitConverter.ToUInt16(serialized, curIndex);
                    curIndex += BitConverter.GetBytes(element).Length;
                    data.Add(element);
                }
                return (curIndex - startIndex);
            }

        }
    }
}
