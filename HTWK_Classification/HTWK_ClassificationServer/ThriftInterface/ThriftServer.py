import logging
logging.basicConfig(level=logging.DEBUG)

import sys
import os

sys.path.append('gen-py')
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/ThriftInterface")

from ThriftInterface import ThriftInterface_Service
from ThriftInterface.ttypes import TDataResult

from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from thrift.server import TServer

from HtwkNet import HtwkNet

class ThriftServiceHandler:
    def __init__(self):
        self.classifier = HtwkNet()

    def loadModel(self, path):
        print('[INFO] loading model ' + path)
        self.classifier.load(path)

    def ping(self, sender):
        print('[INFO] Ping! ' + sender)
        return sender

    def rawData(self, transport_def, raw_data, params):
        # get the stringbyte array with the image data
        transportOut = TTransport.TMemoryBuffer()
        protocolOut = TBinaryProtocol.TBinaryProtocol(transportOut)
        raw_data.write(protocolOut)       
        stringBytes = transportOut.getvalue()

        # unfortunately we have some extra bits before the array when received in python
        # seven bytes are inserted in the byte array, so the image header is invalid.
        # we delete them manually
        stringBytes = stringBytes[7:]

        predictions = self.classifier.classify(stringBytes)
        results = []
        for i in range(0, len(predictions)):
            results.append(TDataResult(i, predictions[i]))

        return results

    def getLabelMap(self, path):
        return self.classifier.get_label_map(labelmap=path)

   
if __name__ == '__main__':
    # start the thirft server
    handler = ThriftServiceHandler()
    processor = ThriftInterface_Service.Processor(handler)
    transport = TSocket.TServerSocket(port=1833)
    tfactory = TTransport.TBufferedTransportFactory()
    pfactory = TBinaryProtocol.TBinaryProtocolFactory()

    server = TServer.TSimpleServer(processor, transport, tfactory, pfactory)

    print('[INFO] Starting the server...')
    server.serve()
    print('[INFO] Server done.')

