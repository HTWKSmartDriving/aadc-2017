#ifndef AADC_USER_VECTORHELPER_H
#define AADC_USER_VECTORHELPER_H
namespace htwk {
    template<typename T>
    inline static void recieveVector(adtf::IMediaSample *pMediaSample, std::vector<T> &vec) {
        {//------------------------------------------
            __adtf_sample_read_lock(pMediaSample, T, pData);
            vec.resize(pMediaSample->GetSize() / sizeof(T));
            memcpy(vec.data(), pData, pMediaSample->GetSize());
        }//------------------
    }
}
#endif //AADC_USER_VECTORHELPER_H
