namespace cpp thrift_interface

/******************************************************************************
 * interface objects
 ******************************************************************************/

/*! Defines what the binary in the DataRaw tranport is carrying
*/
enum TransportDef
{
    IMAGEDATA = 0, //raw data
}

/*! Generic transport struct.
*/
struct TDataRaw
{
    1: required binary raw_data
}

struct TDataId
{
    1: required i64 uuid
}

struct TDataResult
{
    1: required i16 classification
    2: required double probability
}

struct TImageParams
{
    1: required i16 height
    2: required i16 width
    3: required i16 bytesPerPixel
}

typedef list<TDataResult> TDataResultList

struct TLabelMapping
{
    1: required i16 id
    2: required string label
}

typedef list<TLabelMapping> TLabelMap

/** thrown by services */
exception TIoException {
    1: string message;
}

service ThriftInterface_Service {

    /*!Sends a string to a partner, receives one in return.
     * @return the same string
     * @throws TIoException
     */
    string ping(1: string sender) throws (1: TIoException ioe);

    /*!Sends raw byte data, returns a bool upon return.
     * @return the classification results
     */
    TDataResultList rawData(1: TransportDef transport_def,  2: TDataRaw raw_data, 3: TImageParams params) throws (1: TIoException ioe);

    /*!Gets the label mapping from the classifier.
     * @return the labelmap
     */
    TLabelMap getLabelMap(1: string path) throws (1: TIoException ioe);

    /*! Loads the model at the given path
     */
    void loadModel(1: string path) throws (1: TIoException ioe);
}
