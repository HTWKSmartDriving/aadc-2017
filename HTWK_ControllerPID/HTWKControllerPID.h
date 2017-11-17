#ifndef _CONTROLLER_PID_H_
#define _CONTROLLER_PID_H_

#define OID_HTWK_PIDCONTROLLER "htwk.pidcontroller"
#define FILTER_NAME "HTWK PID Controller"


#include "stdafx.h"
#include "../HTWK_Debug/EnableLogs.h"
/*! this is the main class of the controller pid plugin */
/*! Dieser Filter wurde fast komplett aus der AADC Demo übernommen. Er wurde lediglich um eine obere und untere
 * Schranke des Outputs ergänzt.*/
class HTWKControllerPID : public adtf::cFilter {
    /*! This macro does all the plugin setup stuff
    * Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
    */
ADTF_FILTER(OID_HTWK_PIDCONTROLLER, FILTER_NAME, OBJCAT_DataFilter);

    /*! the input pin for the measured value */
    cInputPin m_oInputMeasured;
    /*! the input pin for the set point value */
    cInputPin m_oInputSetPoint;
    /*! the output pin for the manipulated value */
    cOutputPin m_oOutputManipulated;

public:
    /*! constructor for class
    *    \param __info   [in] This is the name of the filter instance.
    */
    HTWKControllerPID(const tChar *__info);

    /*! Destructor. */
    virtual ~HTWKControllerPID();

protected: // overwrites cFilter


    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException **__exception_ptr);

    /*! Implements the default cFilter state machine calls. It will be
    *    called automatically by changing the filters state IFilter::State_Ready -> IFilter::State_Running
    *    and can be overwritten by the special filter.
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \return Standard Result Code.
    *    \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *    (see:  section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    *
    */
    tResult Start(ucom::IException **__exception_ptr = NULL);

    /*!  Implements the default cFilter state machine calls. It will be
    *   called automatically by changing the filters state IFilter::State_Running -> IFilter::State_Ready
    *   and can be overwritten by the special filter.
    *   \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *   If not using the cException smart pointer, the interface has to
    *   be released by calling Unref().
    *   \return Standard Result Code.
    *   \note This method will be also called during the shutdown of a configuration if the Filter is connected to the Message Bus.
    *   (see: section_message_bus)! This has to be done, to disconnect the Message Bus and to avoid Race Conditions.
    */
    tResult Stop(ucom::IException **__exception_ptr = NULL);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *                                   If not using the cException smart pointer, the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
    *   \return Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException **__exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);

private:
    /*! creates all the output Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateOutputPins(ucom::IException **__exception_ptr = NULL);

    /*! creates all the input Pins
    * \param __exception_ptr the exception pointer
    * \return standard adtf error code
    */
    tResult CreateInputPins(ucom::IException **__exception_ptr = NULL);

    /*! calculates the manipulated value for the given values, it uses the setpoint in m_f32SetPoint
    * \param f32MeasuredValue    the measuredValue
    * \return the controller value
    */
    const tFloat32 getControllerValue(const tFloat32 &f32MeasuredValue);

    /*!
    * Gets the time.
    *
    * \return  The streamtime in milliseconds
    */
    tTimeStamp GetTime();

    /*! holds the last measuredValue */
    tFloat32 m_f32MeasuredVariable;
    /*! holds the last measured error */
    tFloat32 m_f32LastMeasuredError;
    /*! holds the last setpoint */
    tFloat32 m_f32SetPoint;
    /*! holds the last sample time */
    tTimeStamp m_lastSampleTime;
    /*! holds the accumulatedVariable for the controller*/
    tFloat32 m_f32AccumulatedVariable;

    /*! Coder Descriptor for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignal;
    /*! the id for the f32value of the media description for the signal value input pins */
    tBufferID m_szIDSignalF32Value;
    /*! the id for the arduino time stamp of the media description for the signal value input pins */
    tBufferID m_szIDSignalArduinoTimestamp;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsSignalSet;

};
/*! @} */ // end of group
#endif // _CONTROLLER_PID_H_

