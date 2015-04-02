#include "recognition.hpp"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace emorph::ehist;
using namespace emorph::reco;

recognition::recognition()
:saccd(0), htwin(0)
{
    init();
}

recognition::recognition(std::string _file, unsigned int _sd, unsigned int _htw, std::string _eye)
:knowledgeFileList(_file), saccd(_sd), htwin(_htw)
{
    if(!_eye.compare("left"))
        eyeSel=0;
    else
        eyeSel=1;

    cout << "[recognition] init" << endl;
    init();
    knowledgeSize=0;
    //load the bases;
    cout << "[recognition] load the base of knowledge" << endl;
    loadKnowledgeBase();
    cout << "[recognition] Base loaded, knowledgeSize: " << knowledgeSize << endl;

/*    cout << "The number of kernel recover form the file for a negative polarity: " << nnegk << endl;
    cout << "Number of negative stv associated of the first model: " << nnegh[0] << endl
         << "\tFirst: " << ptr_negh[0] << " at ts " << ptr_negh[1] << " and addr " << ptr_negh[2] << ":" << ptr_negh[3] << endl
         << "\tLast: "  << ptr_negh[4*(nnegh[0]-1)] << " at ts " << ptr_negh[4*(nnegh[0]-1)+1] << " and addr " << ptr_negh[4*(nnegh[0]-1)+2] << ":" << ptr_negh[4*(nnegh[0]-1)+3] << endl;
    cout << "The number of kernel recover form the file for a positive polarity: " << nposk << endl;
    cout << "Number of positive stv associated of the first model: " << nposh[0] << endl
         << "\tFirst: " << ptr_posh[0] << " at ts " << ptr_posh[1] << " and addr " << ptr_posh[2] << ":" << ptr_posh[3] << endl
         << "\tLast: "  << ptr_posh[4*(nposh[0]-1)] << " at ts " << ptr_posh[4*(nposh[0]-1)+1] << " and addr " << ptr_posh[4*(nposh[0]-1)+2] << ":" << ptr_posh[4*(nposh[0]-1)+3] << endl;
//         << "\tLast: "  << ptr_posh[4*(nposh[0])] << " at ts " << ptr_posh[4*(nposh[0])+1] << " and addr " << ptr_posh[4*(nposh[0])+2] << ":" << ptr_posh[4*(nposh[0])+3] << endl;
*/    
    nBin=static_cast<unsigned int>(ceil(static_cast<double>(saccd)/static_cast<double>(htwin)));
//    if(nBin<1)
//        cerr << "Error: Histogram Time Window greater than the Saccade itself" << endl;
    unsigned int nElem;
    unsigned int index=0;
    ptr_posdh=new Matrix*[sznposh];
    ptr_pos3ddh=new int*[sznposh];
    cout << "[recognition] nposk: " << nposk << ", nBin: " << nBin << endl;
    for(unsigned int i=0; i<sznposh; i++)
    {
        cout << "[recognition] Create dense histogram of positive channel of the object no " << i << endl;
        ptr_posdh[i]=new Matrix(nposk, nBin);
        createDenseHistogram(ptr_posh+index, nposh[i], ptr_posdh[i]);
        
        ptr_pos3ddh[i]=new int[nBin*nposk*128*2];
        memset(ptr_pos3ddh[i], 0, nBin*nposk*128*2*sizeof(int));
        create3DDenseHistogram(ptr_posh+index, nposh[i], ptr_pos3ddh[i], nposk);

        index+=nposh[i]*4;
        countElemInHist(ptr_posdh[i], nElem);
        cout << "[POS] Total number of positive element in the known matrix[" << i << "]: " << nElem << endl;
    }

    index=0;
    ptr_negdh=new Matrix*[sznnegh];
    ptr_neg3ddh=new int*[sznnegh];
    for(unsigned int i=0; i<sznnegh; i++)
    {
        ptr_negdh[i]=new Matrix(nnegk, nBin);
        createDenseHistogram(ptr_negh+index, nnegh[i], ptr_negdh[i]);
        
        ptr_neg3ddh[i]=new int[nBin*nnegk*128*2];
        memset(ptr_neg3ddh[i], 0, nBin*nnegk*128*2*sizeof(int));
        create3DDenseHistogram(ptr_negh+index, nnegh[i], ptr_neg3ddh[i], nnegk);

        index+=nnegh[i]*4;
        countElemInHist(ptr_negdh[i], nElem);
        cout << "[NEG] Total number of positive element in the known matrix[" << i << "]: " << nElem << endl;
    }
}

recognition::~recognition()
{
    delete[] ptr_unkposdh;
    delete[] ptr_unknegdh;
    delete[] ptr_unkpos3ddh;
    delete[] ptr_unkneg3ddh;

    for(uint32_t i=0; i<sznposh; ++i)
    {
        delete[] ptr_posdh[i];
        delete[] ptr_pos3ddh[i];
    }
    delete[] ptr_posdh;
    delete[] ptr_pos3ddh;

    for(uint32_t i=0; i<sznnegh; ++i)
    {
        delete[] ptr_negdh[i];
        delete[] ptr_neg3ddh[i];
    }
    delete[] ptr_negdh;
    delete[] ptr_neg3ddh;

    delete[] nposh;
    delete[] ptr_posh;
    delete[] nnegh;
    delete[] ptr_negh;
}

void recognition::onRead(eventHistBuffer& _hb)
{
    unsigned int nposuh;
    unsigned int *ptr_posuh=NULL;
    unsigned int nneguh;
    unsigned int *ptr_neguh=NULL;
    cerr << "Copy the buffered histograms" << endl;
    //_hb.get_hist(&ptr_posuh, nposuh, &ptr_neguh, nneguh);
    nposuh=_hb.get_histpsz();
    ptr_posuh=_hb.get_histp();
    nneguh=_hb.get_histnsz();
    ptr_neguh=_hb.get_histn();

    cerr << "size of the positive hist: " << nposuh << endl
         << "tstamp of the first elem of the positive hist: " << ptr_posuh[1] << endl
         << "size of the negative hist: " << nneguh << endl
         << "tstamp of the first elem of the negative hist: " << ptr_neguh[1] << endl;

    cerr << "Store the histograms" << endl;
    set_hist(ptr_posuh, nposuh, ptr_neguh, nneguh);
    cerr << "Compare the known and unknown" << endl;
    compare();
}

void recognition::init()
{
    sznposh=0;
    szptrposh=0;
    nposh=NULL;
    ptr_posh=NULL;

    sznnegh=0;
    szptrnegh=0;
    nnegh=NULL;
    ptr_negh=NULL;
}

void recognition::set_hist(unsigned int* _unkpos, unsigned int _nunkpos, unsigned int* _unkneg, unsigned int _nunkneg)
{
    cout << "[recognition] set hist pos" << endl;
    ptr_unkposdh=new Matrix(nposk, nBin);
    createDenseHistogram(_unkpos, _nunkpos, ptr_unkposdh);

    cout << "[recognition] set hist pos 3d" << endl;
    ptr_unkpos3ddh=new int[nBin*nposk*128*2];
    memset(ptr_unkpos3ddh, 0, nBin*nposk*128*2*sizeof(int));
    create3DDenseHistogram(_unkpos, _nunkpos, ptr_unkpos3ddh, nposk);

    cout << "[recognition] set hist neg" << endl;
    ptr_unknegdh=new Matrix(nnegk, nBin);
    createDenseHistogram(_unkneg, _nunkneg, ptr_unknegdh);

    cout << "[recognition] set hist neg 3d" << endl;
    ptr_unkneg3ddh=new int[nBin*nnegk*128*2];
    memset(ptr_unkneg3ddh, 0, nBin*nnegk*128*2*sizeof(int));
    create3DDenseHistogram(_unkneg, _nunkneg, ptr_unkneg3ddh, nnegk);


    unsigned int nElem;
    cout << "[POS] Dimension of the histogram: " << nposk << "x" << nBin << endl;
    countElemInHist(ptr_unkposdh, nElem);
    cout << "[POS] Total number of positive element in the unknown matrix: " << nElem << endl;

    cout << "[NEG] Dimension of the histogram: " << nnegk << "x" << nBin << endl;
    countElemInHist(ptr_unknegdh, nElem);
    cout << "[NEG] Total number of positive element in the unknown matrix: " << nElem << endl;
} 

void recognition::loadKnowledgeBase()
{
    ifstream fd;
    string dfile;
    fd.open(knowledgeFileList.c_str());
    std::cout << "[recognition] Opening the file of list: " << knowledgeFileList << std::endl;
    if(fd.is_open())
    {
        while(fd.good())
        {
            getline(fd, dfile);
            std::cout << "[recognition] Opening base file: " << dfile << std::endl;
            loadFile(dfile);
            knowledgeSize++;
        }
        knowledgeSize/=2;
        fd.close();
    }
    else
        std::cout << "Error: File not found" << std::endl;
}

void recognition::compare()
{
    double interHistdistance=0;
    double *distpos=new double[sznposh];
    double *distneg=new double[sznnegh];

    Vector inter3DSimPos(nposk);
    double *simpos=new double[sznposh];
    double *simneg=new double[sznnegh];
    for(unsigned int i=0; i<sznposh; i++)
    {
        histDist(ptr_unkposdh, ptr_posdh[i], interHistdistance);
        distpos[i]=interHistdistance;
        cout << "[POS] Distance between the unknown entry and known[" << i << "]: " << interHistdistance;

        hist3DDist(ptr_unkpos3ddh, ptr_pos3ddh[i], nposk, inter3DSimPos);
        simpos[i]=1;
        for(unsigned int ii=0; ii<nposk; ++ii)
            simpos[i]*=inter3DSimPos[ii];
        cout << ", with space: " << simpos[i] << endl;
    }
    for(unsigned int i=0; i<sznnegh; i++)
    {
        histDist(ptr_unknegdh, ptr_negdh[i], interHistdistance);
        distneg[i]=interHistdistance;
        cout << "[NEG] Distance between the unknown entry and known[" << i << "]: " << interHistdistance << endl;
    }
    int szBuffer=sznposh+sznnegh;
    double* buffer=new double[szBuffer];
    memcpy(buffer, distpos, sznposh*sizeof(double));
    memcpy(buffer+sznposh, distneg, sznnegh*sizeof(double));

    objDistBuffer outBuf(buffer, szBuffer, eyeSel);
    objDistBuffer& tmp = outPort->prepare();
    tmp=outBuf;
    outPort->write();

    delete[] buffer;
    delete[] distpos;
    delete[] distneg;
    delete[] simpos;
    delete[] simneg;
}

int recognition::loadFile(string _fileToLoad)
{
//    cout << "\t\tLoad file: " << _fileToLoad << endl;

    size_t underscorePosition=_fileToLoad.find_last_of('_');
    size_t periodPosition=_fileToLoad.find_last_of('.');
    if(underscorePosition==string::npos||periodPosition==string::npos)
        return -1;
    string pol=_fileToLoad.substr(underscorePosition+1, (periodPosition-(underscorePosition+1)));
//    cout << "\t\tRecognize polarity: " << pol << endl;

    if(strcmp(pol.c_str(), "neg"))
        loadFileFromPol(_fileToLoad, &ptr_posh, &nposh, sznposh, szptrposh, nposk);
    else if(strcmp(pol.c_str(), "pos"))
        loadFileFromPol(_fileToLoad, &ptr_negh, &nnegh, sznnegh, szptrnegh, nnegk);
}

int recognition::loadFileFromPol(string& _fileToLoad, unsigned int **_ptrh, unsigned int **_nh, unsigned int& _sznh, unsigned int& _szptrh, unsigned int& _nk)
{
    unsigned int nh;
    unsigned int *tmp_ptrh;
    unsigned int *tmp_nh;

    ifstream fd;
    fd.open(_fileToLoad.c_str(), ios::in|ios::app);
    if(fd.is_open())
    {
        fd.read(reinterpret_cast<char*>(&_nk), sizeof(int));
        fd.read(reinterpret_cast<char*>(&nh), sizeof(int));
    //    cout << "\t\t\tSize of the current file: " << nh << endl;
        if(_szptrh)
        {
            tmp_ptrh=new unsigned int[_szptrh];
            memcpy(tmp_ptrh, *_ptrh, _szptrh*sizeof(int));

            tmp_nh=new unsigned int[_sznh];
            memcpy(tmp_nh, *_nh, _sznh*sizeof(int));

            delete[] *_ptrh;
            *_ptrh=new unsigned int[4*nh+_szptrh];
            memcpy(*_ptrh, tmp_ptrh, _szptrh*sizeof(int));

            delete[] *_nh;
            *_nh=new unsigned int[_sznh+1];
            memcpy(*_nh, tmp_nh, _sznh*sizeof(int));
            
            delete[] tmp_ptrh;
            delete[] tmp_nh;
        }
        else
        {
            *_ptrh=new unsigned int[4*nh];
            *_nh=new unsigned int[1];
        }
        tmp_ptrh=new unsigned int[4*nh];
        fd.read(reinterpret_cast<char*>(tmp_ptrh), 4*nh*sizeof(int));
    //    cout << "\t\t\tData size: " << 4*nh*sizeof(int) << endl;

        memcpy(*_ptrh+_szptrh, tmp_ptrh, 4*nh*sizeof(int));
        memcpy(*_nh+_sznh, &nh, sizeof(int));
        _szptrh+=4*nh;
        _sznh++;
    //    cout << "\t\t\tNumber of file load for this pol: " << _sznh << endl;
        delete[] tmp_ptrh;
    }
    else
        cout << "[recognition] Error: file " << _fileToLoad << " not found" << endl; 
}

int recognition::createDenseHistogram(unsigned int* _ptrh, unsigned int _nh, Matrix *_ptrdh)
{
    cerr << "[recognition] recognition::createDenseHistogram(...)" << endl;
    fprintf(stderr, "[recognition] Address of the target of the pointer: %08x\n", _ptrh); 
    unsigned int refts=_ptrh[1];
    fprintf(stderr, "[recognition] ts reference: %d\n", refts);
    unsigned int bin=0;
    _ptrdh->zero();
    cout << "[recognition] Number of dimension: " << _nh << endl;
    for(unsigned int i=0; i<_nh; i++)
    {
        //fprintf(stderr, "\tstep %d / %d\n", i, _nh);
        if(refts+htwin<=_ptrh[4*i+1])
        {
            bin++;
            refts=_ptrh[4*i+1];
        }
        if(bin<_ptrdh->cols())
            (*_ptrdh)(_ptrh[4*i], bin)++;
    }
}

int recognition::create3DDenseHistogram(unsigned int *_ptrh, unsigned int &_nh, int *_ptr3ddh, unsigned int &_nk)
{
    //_ptrh point   0. The kernel ID
    //              1. The ts of the K arrival
    //              2. The K center coordinate in X
    //              3. The K center coordinate in Y

    //Calculation of the center of the object considering the K coordinate
    double cx=0;
    double cxElem=0;
    double cy=0;
    double cyElem=0;
    
    for(unsigned int i=0; i<_nh; ++i)
    {
        cxElem++;
        cyElem++;
        //cout << "[recognition] Center of the current features: " << _ptrh[4*i+2] << ", " << _ptrh[4*i+3] << endl;
        cx=cx+(1/cxElem)*((double)_ptrh[4*i+2]-cx);
        cy=cy+(1/cyElem)*((double)_ptrh[4*i+3]-cy);
    }
    cout << "[recognition] Center of the current shape: " << cx << ", " << cy << endl;
    //Consider _ptr3ddh be initialized to 0 (memset(...) )

    unsigned int refts=_ptrh[1];
    unsigned int bin=0;
    for(unsigned int i=0; i<_nh; i++)
    {
        //fprintf(stderr, "\tstep %d / %d\n", i, _nh);
        if(refts+htwin<=_ptrh[4*i+1])
        {
            bin+=_nk*2*128;
            refts=_ptrh[4*i+1];
        }
       // if(bin<_ptrdh->cols())
       //    (*_ptrdh)(_ptrh[4*i], bin)++;
        int indX=_ptrh[4*i+2]-(int)floor(cx+0.5)+64;
        int indY=_ptrh[4*i+3]-(int)floor(cy+0.5)+64;
        //cout << "[recognition] Add occurence at: " << indX << ", " << indY << endl;

        *(_ptr3ddh+(bin+_ptrh[4*i]*256+indX))+=1;
        *(_ptr3ddh+(bin+_ptrh[4*i]*256+indY+128))+=1;
    }
    
}

void recognition::max(Matrix* _mat, double& _val)
{
    _val=0; 
    for(unsigned int i=0; i<_mat->cols(); i++)
        if(findMax(_mat->getCol(i))>_val)
            _val=findMax(_mat->getCol(i));
}

void recognition::min(Matrix* _mat, double& _val)
{
    _val=1E6; 
    for(unsigned int i=0; i<_mat->cols(); i++)
        if(findMin(_mat->getCol(i))<_val)
            _val=findMin(_mat->getCol(i));
}

void recognition::mean(Vector* _vec, double& _mean)
{
    _mean=0;
    double* dat=_vec->data();
    size_t sz=_vec->size();
    for(unsigned int i=0; i<sz; i++)
        _mean+=*(dat+i);
    _mean/=sz;
//    printVector(_vec);
//    cout << "\tRes of the mean computation: " << _mean << endl;
}

int recognition::cov(Vector* _vec1, Vector* _vec2, double& _cov)
{
    _cov=0;
    double mean1; mean(_vec1, mean1);
    double mean2; mean(_vec2, mean2);
    double *datvec1=_vec1->data();
    double *datvec2=_vec2->data();

    size_t sz=_vec1->size();
    if(_vec2->size()!=sz)
        return 0;
    for(unsigned int e1=0; e1<sz; e1++)
        _cov+=(*(datvec1+e1)-mean1)*(*(datvec2+e1)-mean2);
//        for(unsigned int e2=0; e2<sz; e2++)
//           _cov+=(*(datvec1+e1)-mean1)*(*(datvec2+e2)-mean2);
    _cov/=(sz-1);
//    printVector(_vec1);
//    printVector(_vec2);
//    cout << "\tRes of the cov computation: " << _cov << endl;
    return 1;
}

int recognition::mahalanobisDist(Vector *_vec1, Vector *_vec2, double& _val)
{
    double covariance;
    cov(_vec1, _vec2, covariance);
    _val=sqrt(dot(*(_vec1)*(1/covariance),*(_vec2)));
//    printVector(_vec1);
//    printVector(_vec2);
//    cout << "\tRes of the Mahalanobis distance computation: " << _val << endl;
    return 1;    
}

int recognition::euclideanDist(Vector *_vec1, Vector *_vec2, double& _val)
{
    _val=sqrt(dot(*(_vec1)-*(_vec2),*(_vec1)-*(_vec2)));
//    printVector(_vec1);
//    printVector(_vec2);
//    cout << "\tRes of the Mahalanobis distance computation: " << _val << endl;
    return 1;    
}

void recognition::histDist(Matrix *_hist1, Matrix *_hist2, double& _dist)
{
    _dist=0;
    double step=0;
    Vector vec1(_hist1->rows());
    Vector vec2(_hist1->rows());
    for(unsigned int bin=0; bin<_hist1->cols(); bin++)
    {
        vec1=_hist1->getCol(bin);
        vec2=_hist2->getCol(bin);
        //mahalanobisDist(&vec1, &vec2, step);
        euclideanDist(&vec1, &vec2, step);
        _dist+=step;
    }
}

void recognition::hist3DDist(int *_3DHist1, int *_3DHist2, unsigned int &_nk, Vector &_sim)
{
    unsigned int bin=0;
    _sim.zero();
    double subSim;
    for(unsigned int i=0; i<nBin; i++)
    {
        bin=i*_nk*2*128;
        for(unsigned int ii=0; ii<_nk; ii++)
        {
            cosine(_3DHist1+(bin+ii*256), _3DHist2+(bin+ii*256), subSim);
            cout << "[POS] Bin " << i << ", kernel " << ii << ", result of the cosine: " << subSim << endl;
            _sim(ii)+=subSim;
        }
    }
    _sim/=(double)nBin;
}

void recognition::cosine(int *_sub3DHist1, int *_sub3DHist2, double &_sim)
{
    // A°B/||A||*||B||
    double normA=0;
    double normB=0;
    double AdotB=0;
    for(int i=0; i<256; ++i)
    {
        if(_sub3DHist1[i] || _sub3DHist2[i])
            cout << "[POS] Values: _sub3DHist1[" << i << "]: " << _sub3DHist1[i] << ", _sub3DHist2[" << i << "]: " << _sub3DHist2[i] << endl;
        normA+=(double)_sub3DHist1[i];
        normB+=(double)_sub3DHist2[i];
        AdotB+=(double)_sub3DHist1[i]*(double)_sub3DHist2[i];
    }
    if(normA==0 && normB==0)
        _sim=1;
    else if(normA==0 || normB==0)
        _sim=0;
    else
        _sim=AdotB/(sqrt(normA)*sqrt(normB));
}

void recognition::printVector(yarp::sig::Vector* _vec)
{
    cout << "vec: ";
    double *dat=_vec->data();
    for(unsigned int e=0; e<_vec->size(); e++)
        cout << *(dat+e) << " ";
    cout << endl;
}

void recognition::countElemInHist(Matrix* _mat, unsigned int& _val)
{
    _val=0;
    double *dat=_mat->data();
    size_t nr=_mat->rows();
    size_t nc=_mat->cols();
    for(unsigned int r=0; r<nr; r++)
        for(unsigned int c=0; c<nc; c++)
            if(*(dat+(r*nc)+c)!=0)
                _val++;
}

