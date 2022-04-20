#ifndef MAGLOCALIZERPRO_H
#define MAGLOCALIZERPRO_H

#include <QObject>
#include "filterutils.hpp"
#include "magsensorparams.hpp"

/**
 * @brief MTK_BUILD_MANIFOLD
 * Magic, keep it!!
 */
MTK_BUILD_MANIFOLD (magBasic,
(( vec3 , bigMagPos))
(( SO3 , bigMagOrient))
(( vec3, bigMagVel))
(( vec1, bigMagMoment))
(( vec3, capPos))
(( vec3, capDeltaPos))
(( SO3, capOrient))
(( vec3, capVel))
)
struct magNew:public magBasic
{
    typedef magNew self;
    self& operator=(const self&other)
    {
        bigMagPos = other.bigMagPos;
        bigMagOrient = other.bigMagOrient;
        bigMagVel = other.bigMagVel;
        bigMagMoment = other.bigMagMoment;
        capPos = other.capPos;
        capDeltaPos = other.capDeltaPos;
        capVel = other.capVel;
        capOrient = other.capOrient;
        //capDeltaOrient = other.capDeltaOrient;
        return *this;
    }
};
typedef ukfom::mtkwrap<magNew> magState;

class MagLocalizerPro : public QObject
{
    Q_OBJECT
public:
    typedef MTK::vect<MagSensorParams::NumOfFluxDataInTotal, double> BuildMeasurement;
    explicit MagLocalizerPro(QObject *parent = 0,
                             MagSensorParams *params =
                             new MagSensorParams());
    void updateBigMagState(float x, float y, float z, matXdArgConst& rawData);
    MatrixXd createFlux(double x, double y, double z,
                        double m,
                        double mx, double my, double mz,
                        matXdArgConst&sensorLoc);
    void setTimeInterval(double dt){this->dt = dt;}
    void start(const BuildMeasurement& measurement);
    MatrixXd getRawFlux()const
    {
        return rawFlux;
    }

    MatrixXd getRawDeltaFlux()const
    {
        return deltaRawFlux;
    }

    MatrixXd getEstimateDeltaFlux()const
    {
        return deltaEstimateFlux;
    }

private:
    enum{
        dimMeasurement = MagSensorParams::NumOfFluxDataInTotal
    };
    typedef Differentator<MagSensorParams::NumOfFluxDataInTotal> GradientSolver;
    GradientSolver *gradienter;

    bool isReadyToCalibrate;

    //ukf
    ukfom::ukf<magState> *kf;
    magState initState;

    static magState ukf_fx(const magState &s);
    static BuildMeasurement ukf_hx(const magState &s);
    static ukfom::ukf<magState>::cov ukf_Q();
    static MatrixXd ukf_R();

    static MatrixXd sensorLoc;
    static MatrixXd rawFlux;
    static MatrixXd deltaRawFlux;
    static MatrixXd deltaEstimateFlux;

    static const int scale = 1e6;
    static double bigMagMoment;
    static double capMoment;
    static double dt;

    static Vector3d dpos_bigMag;
    static Vector3d pos_bigMag;

signals:

public slots:
};

#endif // MAGLOCALIZERPRO_H
