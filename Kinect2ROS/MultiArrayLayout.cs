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
        public class MultiArrayLayout : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "MultiArrayLayout";
            public readonly string md5sum = "0fed2a11c13e11c5571b4e2a995a91a3";
            public List<std_msgs.MultiArrayDimension> dim;
            public System.UInt32 data_offset;

            public MultiArrayLayout ()
            {
                dim = new List<std_msgs.MultiArrayDimension>();
                data_offset = 0;
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += "dim:\n[";
                foreach (std_msgs.MultiArrayDimension element in dim)
                {
                    string_rep += " " + element.ToStringRaw();
                }
                string_rep += "]\n\n";
                string_rep += "data_offset: " + Convert.ToString(data_offset) + "\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                System.Byte[] dim_len_bytes = BitConverter.GetBytes((System.UInt32)dim.Count);
                stream.Write(dim_len_bytes, 0, dim_len_bytes.Length);
                foreach(std_msgs.MultiArrayDimension element in dim)
                {
                    element.Serialize(stream);
                }
                System.Byte[] data_offset_bytes = BitConverter.GetBytes(data_offset);
                stream.Write(data_offset_bytes, 0, data_offset_bytes.Length);
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                System.UInt32 dim_len = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(dim_len).Length;
                for (int i = 0; i < (int)dim_len; i++)
                {
                    std_msgs.MultiArrayDimension element = new std_msgs.MultiArrayDimension();
                    curIndex += element.Deserialize(serialized, curIndex);
                    dim.Add(element);
                }
                data_offset = BitConverter.ToUInt32(serialized, curIndex);
                curIndex += BitConverter.GetBytes(data_offset).Length;
                return (curIndex - startIndex);
            }

        }
    }
}
