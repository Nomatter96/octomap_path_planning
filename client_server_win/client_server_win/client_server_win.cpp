#include "client_server_win.h"

using namespace std;

namespace cl {

void
PCLClient::HandleConnect(const boost::system::error_code& aError) 
{
    if (aError)
        return;
    while (!mViewer.wasStopped()) {
        boost::asio::read(mSocket, boost::asio::buffer(&mNrPoints, sizeof(mNrPoints)));
        if (mNrPoints) {
            mBuf->points.resize(mNrPoints);
            boost::asio::read(mSocket, boost::asio::buffer(&mBuf->points.front(), mNrPoints * 8 * sizeof(float)));
            mViewer.showCloud(mBuf);
        }
        mNrPoints = 0;
    }
};

}