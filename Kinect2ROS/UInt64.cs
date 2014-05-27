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
        public class UInt64 : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "UInt64";
            public readonly string md5sum = "1b2a79973e8bf53d7b53acb71299cb57";
            public System.UInt64 data;

            public UInt64 ()
            {
                data = 0;
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += "data: " + Convert.ToString(data) + "\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                System.Byte[] data_bytes = BitConverter.GetBytes(data);
                stream.Write(data_bytes, 0, data_bytes.Length);
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                data = BitConverter.ToUInt64(serialized, curIndex);
                curIndex += BitConverter.GetBytes(data).Length;
                return (curIndex - startIndex);
            }

        }
    }
}