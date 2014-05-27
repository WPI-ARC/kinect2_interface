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
    namespace geometry_msgs
    {
        public class Quaternion : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "Quaternion";
            public readonly string md5sum = "a779879fadf0160734f906b8c19c7004";
            public double x;
            public double y;
            public double z;
            public double w;

            public Quaternion ()
            {
                x = 0.0;
                y = 0.0;
                z = 0.0;
                w = 0.0;
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += "x: " + Convert.ToString(x) + "\n";
                string_rep += "y: " + Convert.ToString(y) + "\n";
                string_rep += "z: " + Convert.ToString(z) + "\n";
                string_rep += "w: " + Convert.ToString(w) + "\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                System.Byte[] x_bytes = BitConverter.GetBytes(x);
                stream.Write(x_bytes, 0, x_bytes.Length);
                System.Byte[] y_bytes = BitConverter.GetBytes(y);
                stream.Write(y_bytes, 0, y_bytes.Length);
                System.Byte[] z_bytes = BitConverter.GetBytes(z);
                stream.Write(z_bytes, 0, z_bytes.Length);
                System.Byte[] w_bytes = BitConverter.GetBytes(w);
                stream.Write(w_bytes, 0, w_bytes.Length);
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                x = BitConverter.ToDouble(serialized, curIndex);
                curIndex += BitConverter.GetBytes(x).Length;
                y = BitConverter.ToDouble(serialized, curIndex);
                curIndex += BitConverter.GetBytes(y).Length;
                z = BitConverter.ToDouble(serialized, curIndex);
                curIndex += BitConverter.GetBytes(z).Length;
                w = BitConverter.ToDouble(serialized, curIndex);
                curIndex += BitConverter.GetBytes(w).Length;
                return (curIndex - startIndex);
            }

        }
    }
}
