using System;

namespace Protocol
{
    public static class Protocol_Security
    {
        public static short CheckFunctionLenght(ushort msgFunction)
        {
            switch (msgFunction)
            {
                // -2               : UNKNOW
                // -1               : UNLIMITED 
                // [0:MAX_LENGHT]   : FIXED
                
                default:
                    return -2;
            }
        }
    }
}
