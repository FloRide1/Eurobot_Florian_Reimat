using EventArgsLibrary;
using System;
using Constants;
using Protocol_Security;

namespace MessageDecoder
{
    public class MsgDecoder
    {
        public MsgDecoder()
        {

        }

        //byte CalculateChecksum(int msgFunction,
        //        int msgPayloadLength, byte[] msgPayload)
        //{
        //    byte checksum = 0;
        //    checksum ^= (byte)(msgFunction >> 8);
        //    checksum ^= (byte)(msgFunction >> 0);
        //    checksum ^= (byte)(msgPayloadLength >> 8);
        //    checksum ^= (byte)(msgPayloadLength >> 0);
        //    for (int i = 0; i < msgPayloadLength; i++)
        //    {
        //        checksum ^= msgPayload[i];
        //    }
        //    return checksum;
        //}

        //public enum StateReception
        //{
        //    Waiting,
        //    FunctionMSB,
        //    FunctionLSB,
        //    PayloadLengthMSB,
        //    PayloadLengthLSB,
        //    Payload,
        //    CheckSum
        //}

        //StateReception rcvState = StateReception.Waiting;
        //int msgDecodedFunction = 0;
        //int msgDecodedPayloadLength = 0;
        //byte[] msgDecodedPayload;
        //int msgDecodedPayloadIndex = 0;

        //private void DecodeMessage(byte c)
        //{
        //    switch (rcvState)
        //    {
        //        case StateReception.Waiting:
        //            if (c == 0xFE)
        //                rcvState = StateReception.FunctionMSB;
        //            break;
        //        case StateReception.FunctionMSB:
        //            msgDecodedFunction = (Int16)(c << 8);
        //            rcvState = StateReception.FunctionLSB;
        //            break;
        //        case StateReception.FunctionLSB:
        //            msgDecodedFunction += (Int16)(c << 0);
        //            rcvState = StateReception.PayloadLengthMSB;
        //            break;
        //        case StateReception.PayloadLengthMSB:
        //            msgDecodedPayloadLength = (Int16)(c << 8);
        //            rcvState = StateReception.PayloadLengthLSB;
        //            break;
        //        case StateReception.PayloadLengthLSB:
        //            msgDecodedPayloadLength += (Int16)(c << 0);
        //            if (msgDecodedPayloadLength > 0)
        //            {
        //                if (msgDecodedPayloadLength < 1024)
        //                {
        //                    msgDecodedPayloadIndex = 0;
        //                    msgDecodedPayload = new byte[msgDecodedPayloadLength];
        //                    rcvState = StateReception.Payload;
        //                }
        //                else
        //                {
        //                    rcvState = StateReception.Waiting;
        //                }
        //            }
        //            else
        //                rcvState = StateReception.CheckSum;
        //            break;
        //        case StateReception.Payload:
        //            msgDecodedPayload[msgDecodedPayloadIndex++] = c;
        //            if (msgDecodedPayloadIndex >= msgDecodedPayloadLength)
        //            {
        //                rcvState = StateReception.CheckSum;
        //            }
        //            break;
        //        case StateReception.CheckSum:
        //            byte calculatedChecksum = CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
        //            byte receivedChecksum = c;
        //            if (calculatedChecksum == receivedChecksum)
        //            {
        //                //Lance l'event de fin de decodage
        //                OnMessageDecoded(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
        //            }
        //            else
        //            {
        //                OnMessageDecodedError(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
        //            }
        //            rcvState = StateReception.Waiting;
        //            break;
        //        default:
        //            rcvState = StateReception.Waiting;
        //            break;
        //    }
        //}


        ////Input CallBack        
        //public void DecodeMsgReceived(object sender, DataReceivedArgs e)
        //{
        //    foreach (var b in e.Data)
        //    {
        //        DecodeMessage(b);
        //    }
        //}

        ////Output Events
        //public delegate void MessageDecodedEventHandler(object sender, MessageDecodedArgs e);
        //public event EventHandler<MessageDecodedArgs> OnMessageDecodedEvent;
        //public virtual void OnMessageDecoded(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        //{
        //    var handler = OnMessageDecodedEvent;
        //    if (handler != null)
        //    {
        //        handler(this, new MessageDecodedArgs { MsgFunction = msgFunction, MsgPayloadLength = msgPayloadLength, MsgPayload = msgPayload});
        //    }
        //}


        //public delegate void MessageDecodedErrorEventHandler(object sender, MessageDecodedArgs e);
        //public event EventHandler<MessageDecodedArgs> OnMessageDecodedErrorEvent;
        //public virtual void OnMessageDecodedError(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        //{
        //    var handler = OnMessageDecodedErrorEvent;
        //    if (handler != null)
        //    {
        //        handler(this, new MessageDecodedArgs { MsgFunction = msgFunction, MsgPayloadLength = msgPayloadLength, MsgPayload = msgPayload });
        //    }
        //}

        private enum State
        {
            Waiting,
            FunctionMSB,
            FunctionLSB,
            PayloadLengthMSB,
            PayloadLengthLSB,
            Payload,
            CheckSum
        }

        static State actualState = State.Waiting;

        private static byte functionMSB;
        private static byte functionLSB;
        private static byte payloadLenghtMSB;
        private static byte payloadLenghtLSB;

        private static ushort msgFunction;
        private static ushort msgPayloadLenght;
        private static byte[] msgPayload;
        private static byte msgChecksum;

        private static int msgPayloadIndex = 0; // Maybe edit type
        public void ByteReceived(object sender, byte b)
        {
            switch (actualState)
            {
                case State.Waiting:
                    if (b == ConstVar.START_OF_FRAME)
                    {
                        OnSOFReceived(b);
                    }
                    else
                    {
                        OnUnknowReceived(b);
                    }
                    break;

                case State.FunctionMSB:
                    OnFunctionMSBReceived(b);
                    break;

                case State.FunctionLSB:
                    OnFunctionLSBReceived(b);
                    break;

                case State.PayloadLengthMSB:
                    OnPayloadLenghtMSBReceided(b);
                    break;

                case State.PayloadLengthLSB:
                    OnPayloadLenghtLSBReceided(b);
                    break;

                case State.Payload:
                    OnPayloadByteReceived(b);
                    break;

                case State.CheckSum:
                    OnCheckSumReceived(b);
                    break;
            }

        }

        public event EventHandler<EventArgs> OnMessageDecoderCreatedEvent;
        public event EventHandler<byte> OnSOFByteReceivedEvent;
        public event EventHandler<byte> OnUnknowByteEvent;
        public event EventHandler<byte> OnFunctionMSBByteReceivedEvent;
        public event EventHandler<byte> OnFunctionLSBByteReceivedEvent;
        public event EventHandler<byte> OnPayloadLenghtMSBByteReceivedEvent;
        public event EventHandler<byte> OnPayloadLenghtLSBByteReceivedEvent;
        public event EventHandler<byte> OnPayloadByteReceivedEvent;
        public event EventHandler<DecodePayloadArgs> OnPayloadReceivedEvent;
        public event EventHandler<byte> OnChecksumByteReceivedEvent;
        public event EventHandler<MessageByteArgs> OnCorrectMessageReceivedEvent;
        public event EventHandler<MessageByteArgs> OnErrorMessageReceivedEvent;
        public event EventHandler<EventArgs> OnOverLenghtMessageEvent;
        public event EventHandler<EventArgs> OnUnknowFunctionEvent;
        public event EventHandler<EventArgs> OnWrongLenghtFunctionEvent;

        public virtual void OnMessageDecoderCreated()
        {
            OnMessageDecoderCreatedEvent?.Invoke(this, new EventArgs());
        }
        public virtual void OnSOFReceived(byte e)
        {
            actualState = State.FunctionMSB;
            OnSOFByteReceivedEvent?.Invoke(this, e);

        }
        public virtual void OnUnknowReceived(byte e)
        {
            OnUnknowByteEvent?.Invoke(this, e);

        }
        public virtual void OnFunctionMSBReceived(byte e)
        {
            functionMSB = e;
            msgFunction = (ushort)(e << 8);
            actualState = State.FunctionLSB;
            OnFunctionMSBByteReceivedEvent?.Invoke(this, e);
        }
        public virtual void OnFunctionLSBReceived(byte e)
        {
            functionLSB = e;
            msgFunction += (ushort)(e << 0);
            OnFunctionLSBByteReceivedEvent?.Invoke(this, e);
            if (Protocol_Security.Protocol_Security.CheckFunctionLenght(msgFunction) != -2)
            {
                actualState = State.PayloadLengthMSB;
            }
            else
            {
                actualState = State.Waiting;
                OnUnknowFunction();
            }

        }
        public virtual void OnPayloadLenghtMSBReceided(byte e)
        {
            payloadLenghtMSB = e;
            msgPayloadLenght = (ushort)(e << 8);
            actualState = State.PayloadLengthLSB;
            OnPayloadLenghtMSBByteReceivedEvent?.Invoke(this, e);
        }
        public virtual void OnPayloadLenghtLSBReceided(byte e)
        {
            payloadLenghtLSB = e;
            msgPayloadLenght += (ushort)(e << 0);
            actualState = State.Waiting;
            OnPayloadLenghtLSBByteReceivedEvent?.Invoke(this, e);
            if (msgPayloadLenght <= ConstVar.MAX_MSG_LENGHT)
            {
                short allowedLenght = Protocol_Security.Protocol_Security.CheckFunctionLenght(msgFunction);
                if (allowedLenght != -2)
                {
                    if (allowedLenght == -1 || allowedLenght == msgPayloadLenght)
                    {
                        actualState = State.Payload;
                        msgPayloadIndex = 0;
                        msgPayload = new byte[msgPayloadLenght];
                    }
                    else
                    {
                        OnWrongLenghtFunction();
                    }
                }
                else
                {
                    OnUnknowFunction();
                }

            }
            else
            {
                OnOverLenghtMessage();
            }

        }



        public virtual void OnOverLenghtMessage()
        {
            OnOverLenghtMessageEvent?.Invoke(this, new EventArgs());
        }
        public virtual void OnUnknowFunction()
        {
            OnUnknowFunctionEvent?.Invoke(this, new EventArgs());
        }
        public virtual void OnWrongLenghtFunction()
        {
            OnWrongLenghtFunctionEvent?.Invoke(this, new EventArgs());
        }
        public virtual void OnPayloadByteReceived(byte e)
        {
            msgPayload[msgPayloadIndex] = e;
            msgPayloadIndex++;
            if (msgPayloadIndex == msgPayloadLenght)
            {
                OnPayloadReceived(msgPayload);
            }
            OnPayloadByteReceivedEvent?.Invoke(this, e);
        }
        public virtual void OnPayloadReceived(byte[] e)
        {
            actualState = State.CheckSum;
            OnPayloadReceivedEvent?.Invoke(this, new DecodePayloadArgs(e));
        }
        public virtual void OnCheckSumReceived(byte e)
        {
            msgChecksum = e;
            if (msgChecksum == CalculateChecksum())
            {
                OnCorrectMessageReceived();
            }
            else
            {
                OnErrorMessageReceived();
            }
            actualState = State.Waiting;
            OnChecksumByteReceivedEvent?.Invoke(this, e);
        }
        public virtual void OnCorrectMessageReceived()
        {
            OnCorrectMessageReceivedEvent?.Invoke(this, new MessageByteArgs(msgFunction, msgPayloadLenght, msgPayload, msgChecksum));
        }
        public virtual void OnErrorMessageReceived()
        {
            OnErrorMessageReceivedEvent?.Invoke(this, new MessageByteArgs(msgFunction, msgPayloadLenght, msgPayload, msgChecksum));
        }
        private static byte CalculateChecksum()
        {
            byte checksum = ConstVar.START_OF_FRAME;
            checksum ^= functionMSB;
            checksum ^= functionLSB;
            checksum ^= payloadLenghtMSB;
            checksum ^= payloadLenghtLSB;
            foreach (byte x in msgPayload)
            {
                checksum ^= x;
            }
            return checksum;
        }

        
       
    }
}
