#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/nr-module.h"


#include "ns3/internet-module.h"
#include "ns3/config-store-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-module.h"
#include "ns3/antenna-module.h"

using namespace ns3; 
int numCells = 8; // matches total_gnb

#include <chrono>
auto t0 = std::chrono::steady_clock::now();
#define CHECKPOINT(label) { \
    auto now = std::chrono::steady_clock::now(); \
    std::cout << label << ": " << std::chrono::duration<double>(now - t0).count() << "s elapsed" << std::endl; \
}

#include <cmath>

AnimationInterface* anim = nullptr;
NodeContainer gnbNodes;
NodeContainer ueNodes;

void
PrintContainerMembers(NodeContainer container, std::string label)
{
    std::cout << "--- " << label << " (" << container.GetN() << " nodes) ---" << std::endl;
    for (uint32_t i = 0; i < container.GetN(); i++)
    {
        Ptr<Node> node = container.Get(i);
        std::cout << "  Container index " << i << " -> Node::GetId() = " << node->GetId() << std::endl;
    }
}

std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> colors = 
{
    {123, 104, 238}, // Medium Slate Blue
    {0, 128, 0}, // Dark Green
    {165, 42, 42}, // Brown
    {128, 128, 128}, // Gray
    {255, 105, 180}, // Hot Pink
    {0, 255, 0}, // Green
    {0, 0, 255}, // Blue
    {255, 0, 0} // Red
    
    // {255, 165, 0}, // Orange
    // {0, 255, 255}, // Cyan
    // {255, 255, 0}, // Yellow
};

void
PrintProgress()
{
    std::cout << "Simulation time: " << Simulator::Now().GetSeconds() << "s" << std::endl;
}

std::tuple<double,double> boundary(double x, double y)
{
    double x_rand, y_rand;

    Ptr<UniformRandomVariable> randAngle = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> randRadius = CreateObject<UniformRandomVariable>();

    double theta = randAngle->GetValue(0, 2 * M_PI);
    double r = 25 * std::sqrt(randRadius->GetValue(0, 1)); 

    x_rand = x + r * std::cos(theta);
    y_rand = y + r * std::sin(theta);

    return std::make_tuple(x_rand, y_rand);
}

void
UpdateColors(NodeContainer ueNodes)
{
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        uint16_t cellId = ueNodes.Get(i)->GetDevice(0)->GetObject<NrUeNetDevice>()->GetRrc()->GetCellId();
        anim->UpdateNodeColor(ueNodes.Get(i), std::get<0>(colors[cellId - 1]), std::get<1>(colors[cellId - 1]), std::get<2>(colors[cellId - 1]));
    }
};

void
RecvMeasurementReportCallback(Ptr<OutputStreamWrapper> stream, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, NrRrcSap::MeasurementReport report)
{
    // uint64_t rawImsi = imsi; // save the original, unadjusted value for the registry check

    std::vector<double> rsrp(numCells + 1, 0);      // 1-indexed by cellId
    std::vector<double> uedistance(numCells + 1, 0);
    std::vector<double> distance(numCells + 1, 0);
    std::vector<Vector> gnbPos(numCells + 1);

    // imsi = imsi - numCells; // Adjust IMSI to match UE index in ueNodes

    for (int c = 1; c <= numCells; c++)
        gnbPos[c] = gnbNodes.Get(c - 1)->GetObject<MobilityModel>()->GetPosition();

    bool haveMeasResultNeighCells = report.measResults.haveMeasResultNeighCells;
    uint8_t serving_rsrp = report.measResults.measResultPCell.rsrpResult;

    double x = 0, y = 0, z = 0;

    if (cellId >= 1 && cellId <= numCells)
    {
        rsrp[cellId] = serving_rsrp;
        x = gnbPos[cellId].x;
        y = gnbPos[cellId].y;
        z = gnbPos[cellId].z;
    }
        


    if (imsi >= ueNodes.GetN())
    {
        std::cout << "WARNING: IMSI " << imsi << " out of range, skipping." << std::endl;
        return;
    }
    
    // --- CROSS-CHECK BLOCK, corrected ---
    // Ptr<NrUeNetDevice> ueDevFromRegistry = NrDeviceRegistry::GetUeNetDevFromImsi(rawImsi); // use rawImsi here
    // Vector positionFromRegistry = ueDevFromRegistry->GetNode()->GetObject<MobilityModel>()->GetPosition();
    // Vector positionFromYourAdjustment = ueNodes.Get(imsi)->GetObject<MobilityModel>()->GetPosition(); // your existing index-based approach

    // std::cout << "Raw IMSI " << rawImsi << " -> adjusted index " << imsi
    //           << " | Registry pos: (" << positionFromRegistry.x << "," << positionFromRegistry.y << ")"
    //           << " | Your adjustment pos: (" << positionFromYourAdjustment.x << "," << positionFromYourAdjustment.y << ")"
    //           << (positionFromRegistry == positionFromYourAdjustment ? " MATCH" : " MISMATCH")
    //           << std::endl;
    // --- END CROSS-CHECK BLOCK ---

    Vector position = ueNodes.Get(imsi)->GetObject<MobilityModel>()->GetPosition();

    int neighbourCount = 0;
    if (haveMeasResultNeighCells)
    {
        for (auto it = report.measResults.measResultListEutra.begin(); it != report.measResults.measResultListEutra.end(); ++it)
        {
            neighbourCount++;
            if (it->physCellId >= 1 && it->physCellId <= numCells)
                rsrp[it->physCellId] = it->rsrpResult;
        }

        *stream->GetStream() << Simulator::Now().GetSeconds() << "\t" << imsi+1 << "\t"
                              << position.x << "\t" << position.y << "\t" << position.z << "\t"
                              << cellId << "\t" << neighbourCount;

        for (int c = 1; c <= numCells; c++)
        {
            double dist = std::sqrt(std::pow(position.x - gnbPos[c].x, 2) +
                                     std::pow(position.y - gnbPos[c].y, 2) +
                                     std::pow(position.z - gnbPos[c].z, 2));
            uedistance[c] = dist;
            double another_dist = std::sqrt(std::pow(x - gnbPos[c].x, 2) +
                                     std::pow(y - gnbPos[c].y, 2) +
                                     std::pow(z - gnbPos[c].z, 2));
            distance[c] = another_dist;

            *stream->GetStream() << "\t" << rsrp[c] << "\t" << gnbPos[c].x << "\t" << gnbPos[c].y << "\t" << gnbPos[c].z << "\t" << another_dist << "\t" << dist;
        }
        *stream->GetStream() << "\n";
    }
}

int main (int argc, char* argv[])
{
    // Right now its coded for generating short time sanity check dataset. need to check simTime and location of BSs

    // For reproducibility, adjust number of UEs and gNBs, and the seed and run values
    double simTime =5.0;
    uint16_t total_gnb = 8;
    uint16_t total_ue = 300;
    double size_x = 1000;
    double size_y = 1000;
    bool enableAnimation = false;
    uint16_t seed = 44;
    uint16_t run = 1;

    double lbs  = 48, fbs = 50;

    bool propagation = false;

    std::string propagationLossModel;
    double hBS;
    double frequency; 


    if (propagation == true) 
    {
        propagationLossModel = "ThreeGppUMaPropagationLossModel";
        hBS = 25;
        frequency = 2500e6; 

    }
    else
    {
        propagationLossModel = "Cost231PropagationLossModel";
        hBS = 30;
        frequency = 2000e6;  
    }

    std::stringstream animStream;
    std::string xmlFileName;
    animStream << "localization propagation="<< propagationLossModel << " time=" << simTime<< " grid=1000X1000 n(UE)=" << total_ue << " n(gNB+FBS)=" << total_gnb << " lbs=" << lbs << " fbs=" << fbs << \
    " seed=" << seed << " run=" << run;
    if (enableAnimation) xmlFileName = animStream.str() + ".xml";
    std::string txtFileName = animStream.str() + ".txt";

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run);

    // As of now, if UEs are introduced before gNBs, there will be no error in IMSI print section in RecvMeasurementReportCallback Function
    ueNodes.Create(total_ue);
    gnbNodes.Create(total_gnb);
    
    // PrintContainerMembers(ueNodes, "ueNodes");
    // PrintContainerMembers(gnbNodes, "gnbNodes");

    std::tuple<double,double> center = {size_x/2,size_y/2};

    std::vector<std::tuple<double,double>> return_values;

    std::vector<double> thetas = {240,300,180,0,120,60};

    Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();

    double x,y;

    int counter = 0;
    
    double fbs_movement_angle;
    double randomness_angle = rand->GetValue(-M_PI/4, M_PI/4); // Random angle between -45 and 45 degrees

    for (int i=0;i<total_gnb;i++)
    {
        // Add the boundary points to the return_values vector
        if (i!=3 && i!=total_gnb-1) 
        {
            return_values.push_back(boundary(std::get<0>(center)+450*std::cos(thetas[counter]*M_PI/180), std::get<1>(center)+450*std::sin(thetas[counter]*M_PI/180)));
            counter++;
        }

        else if (i==3) return_values.push_back(boundary(std::get<0>(center), std::get<1>(center)));

        else if (i==total_gnb-1) 
        {
            double theta = rand->GetValue(0, 2 * M_PI);
            fbs_movement_angle = theta + M_PI + randomness_angle; // Store the movement angle for later use

            x = std::get<0>(center)+225*std::cos(theta);
            y = std::get<1>(center)+225*std::sin(theta);

            return_values.push_back(boundary(x, y));
        }
    }


    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();

    std::vector<std::tuple<double,double,double>> positions;
    if (seed==11)
    {
        positions.push_back(std::make_tuple(287.384, 89.394, hBS));
        // Location of gNB1: ((array([287.384]), array([89.394]), array([30])))
        positions.push_back(std::make_tuple(745.235, 108.413, hBS));
        // Location of gNB2: ((array([745.235]), array([108.413]), array([30])))
        positions.push_back(std::make_tuple(37.6717, 482.891, hBS));
        // Location of gNB3: ((array([37.6717]), array([482.891]), array([30])))
        positions.push_back(std::make_tuple(509.795, 503.665, hBS));
        // Location of gNB4: ((array([509.795]), array([503.665]), array([30])))
        positions.push_back(std::make_tuple(938.333, 492.253, hBS));
        // Location of gNB5: ((array([938.333]), array([492.253]), array([30])))
        positions.push_back(std::make_tuple(278.912, 873.984, hBS));
        // Location of gNB6: ((array([278.912]), array([873.984]), array([30])))
        positions.push_back(std::make_tuple(728.704, 885.666, hBS));
        // Location of gNB7: ((array([728.704]), array([885.666]), array([30])))
    }
    else if (seed==22)
    {
        positions.push_back(std::make_tuple(263.691, 89.6252, hBS));
        // Location of gNB1: ((array([263.691]), array([89.6252]), array([30])))
        positions.push_back(std::make_tuple(738.933, 107.683, hBS));
        // Location of gNB2: ((array([738.933]), array([107.683]), array([30])))
        positions.push_back(std::make_tuple(44.8546, 515.425, hBS));
        // Location of gNB3: ((array([44.8546]), array([515.425]), array([30])))
        positions.push_back(std::make_tuple(511.158, 509.709, hBS));
        // Location of gNB4: ((array([511.158]), array([509.709]), array([30])))
        positions.push_back(std::make_tuple(957.686, 518.254, hBS));
        // Location of gNB5: ((array([957.686]), array([518.254]), array([30])))
        positions.push_back(std::make_tuple(254.751, 878.974, hBS));
        // Location of gNB6: ((array([254.751]), array([878.974]), array([30])))
        positions.push_back(std::make_tuple(724.318, 881.985, hBS));
        // Location of gNB7: ((array([724.318]), array([881.985]), array([30])))
    }
    else if (seed==33)
    {
        positions.push_back(std::make_tuple(252.214, 109.508, hBS));
        // Location of gNB1: ((array([252.214]), array([109.508]), array([30])))
        positions.push_back(std::make_tuple(748.83, 103.507, hBS));
        // Location of gNB2: ((array([748.83]), array([103.507]), array([30])))
        positions.push_back(std::make_tuple(58.7552, 497.268, hBS));
        // Location of gNB3: ((array([58.7552]), array([497.268]), array([30])))
        positions.push_back(std::make_tuple(508.632, 515.926, hBS));
        // Location of gNB4: ((array([508.632]), array([515.926]), array([30])))
        positions.push_back(std::make_tuple(954.524, 476.168, hBS));
        // Location of gNB5: ((array([954.524]), array([476.168]), array([30])))
        positions.push_back(std::make_tuple(266.473, 899.214, hBS));
        // Location of gNB6: ((array([266.473]), array([899.214]), array([30])))
        positions.push_back(std::make_tuple(717.456, 883.937, hBS));
        // Location of gNB7: ((array([717.456]), array([883.937]), array([30])))
    }
    else if (seed==44)
    {
        positions.push_back(std::make_tuple(263.133, 128.833, hBS));
        // Location of gNB1: ((array([263.133]), array([128.833]), array([30])))
        positions.push_back(std::make_tuple(743.691, 103.046, hBS));
        // Location of gNB2: ((array([743.691]), array([103.046]), array([30])))
        positions.push_back(std::make_tuple(31.6085, 486.195, hBS));
        // Location of gNB3: ((array([31.6085]), array([486.195]), array([30])))
        positions.push_back(std::make_tuple(502.89, 520.716, hBS));
        // Location of gNB4: ((array([502.89]), array([520.716]), array([30])))
        positions.push_back(std::make_tuple(941.173, 509.036, hBS));
        // Location of gNB5: ((array([941.173]), array([509.036]), array([30])))
        positions.push_back(std::make_tuple(286.575, 906.79, hBS));
        // Location of gNB6: ((array([286.575]), array([906.79]), array([30])))
        positions.push_back(std::make_tuple(714.2, 891.634, hBS));
        // Location of gNB7: ((array([714.2]), array([891.634]), array([30])))
    }
    else if (seed==55)
    {
        positions.push_back(std::make_tuple(284.538, 129.228, hBS));
        // Location of gNB1: ((array([284.538]), array([129.228]), array([30])))
        positions.push_back(std::make_tuple(737.331, 104.147, hBS));
        // Location of gNB2: ((array([737.331]), array([104.147]), array([30])))
        positions.push_back(std::make_tuple(49.6367, 518.665, hBS));
        // Location of gNB3: ((array([49.6367]), array([518.665]), array([30])))
        positions.push_back(std::make_tuple(494.91, 522.825, hBS));
        // Location of gNB4: ((array([494.91]), array([522.825]), array([30])))
        positions.push_back(std::make_tuple(968.442, 496.051, hBS));
        // Location of gNB5: ((array([968.442]), array([496.051]), array([30])))
        positions.push_back(std::make_tuple(282.471, 886.968, hBS));
        // Location of gNB6: ((array([282.471]), array([886.968]), array([30])))
        positions.push_back(std::make_tuple(718.432, 900.069, hBS));
        // Location of gNB7: ((array([718.432]), array([900.069]), array([30])))
    }
    else
    {
        NS_FATAL_ERROR("No hard-coded BS layout for seed " << seed
                    << " (use 11, 22, 33, 44 or 55)");
    }

    for (int i=0;i<total_gnb;i++) 
    {
        if (i==total_gnb-1) gnbPositionAlloc->Add(Vector(std::get<0>(return_values[i]),std::get<1>(return_values[i]),2.5));
        // else gnbPositionAlloc->Add(Vector(std::get<0>(return_values[i]),std::get<1>(return_values[i]),hBS));
        else gnbPositionAlloc->Add(Vector(std::get<0>(positions[i]),std::get<1>(positions[i]),hBS));
    }

    MobilityHelper gnbMobility;
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);

    for (int i=0;i<total_gnb;i++)
    {
        if (i!=total_gnb-1) gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        else gnbMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");        
        gnbMobility.Install(gnbNodes.Get(i));        
    }

    double speed = 10.0; 
    Vector fbsVelocity(speed * std::cos(fbs_movement_angle), speed * std::sin(fbs_movement_angle), 0.0);
    gnbNodes.Get(total_gnb-1)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(fbsVelocity);


    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    double z;

    for (int i=0;i<total_ue;i++)
    {
        x = rand->GetValue(0, size_x);
        y = rand->GetValue(0, size_y);
        z = rand->GetValue(1.5, 2.5);

        ns3::Vector initiaPosition(x,y,z);
        ueMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                    "Mode", StringValue("Time"),
                                    "Time", TimeValue(Seconds(15.0)),
                                    "Speed", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"),
                                    "Bounds", RectangleValue(Rectangle(0, size_x, 0, size_y)));
        ueMobility.Install(ueNodes.Get(i));
        ueNodes.Get(i)->GetObject<ns3::MobilityModel> ()->SetPosition (Vector(x,y,z));
    }

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    idealBeamformingHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId())); 
    
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    double bandwidth = 20e6;    

    // if (propagation)
    // {
    //     scenario = "UMa";
    //     BandwidthPartInfoPtrVector allBwps;
    //     CcBwpCreator ccBwpCreator;
    //     const uint8_t numCcPerBand = 1; 
    //     CcBwpCreator::SimpleOperationBandConf bandConf(frequency, bandwidth, numCcPerBand);
    //     OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    //     // Create the channel helper
    //     // Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    //     // Set and configure the channel to the current band
    //     std::vector<CcBwpCreator::SimpleOperationBandConf> bandConfs;
    //     bandConfs.push_back(CcBwpCreator::SimpleOperationBandConf(frequency, bandwidth, 1));

    //     auto [totalBandwidth, allBwps] = nrHelper->CreateBandwidthParts(bandConfs, scenario, "Default", "ThreeGpp");

    //     channelHelper->AssignChannelsToBands({band});
    //     allBwps = CcBwpCreator::GetAllBwps({band});
        
    // }

    // else
    // {
    //     CcBwpCreator ccBwpCreator;
    //     OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(CcBwpCreator::SimpleOperationBandConf(frequency, bandwidth, 1));
    
    //     channelHelper->ConfigurePropagationFactory(Cost231PropagationLossModel::GetTypeId());
    //     channelHelper->AssignChannelsToBands({band});
    // }
    CcBwpCreator ccBwpCreator;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(
    CcBwpCreator::SimpleOperationBandConf(frequency, bandwidth, 1));

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    if (propagation)
    {
        std::string scenario = "UMa";
        channelHelper->ConfigureFactories(scenario, "Default", "ThreeGpp");
        channelHelper->AssignChannelsToBands({band});
    }
    else
    {
        channelHelper->ConfigurePropagationFactory(Cost231PropagationLossModel::GetTypeId());
        channelHelper->AssignChannelsToBands({band});
    }

    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    Ptr<Node> sgw = nrEpcHelper->GetSgwNode();
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();

    MobilityHelper coreMobility;
    coreMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Vector forinternet(size_x + 200, 700.0, 0.0); // remoteHost
    Vector forpgw(size_x + 200, 600.0, 0.0);      // PGW
    Vector forsgw(size_x + 200, 500.0, 0.0);      // SGW
    Vector formme(size_x + 200, 400.0, 0.0);      // MME

    coreMobility.Install(pgw);
    coreMobility.Install(sgw);

    Ptr<Node> mme = NodeList::GetNode(2 + total_ue + total_gnb);
    coreMobility.Install(mme);
    mme->GetObject<MobilityModel>()->SetPosition(formme);

    pgw->GetObject<MobilityModel>()->SetPosition(forpgw);
    sgw->GetObject<MobilityModel>()->SetPosition(forsgw);



    // Antennas for the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Antennas for the gNBs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetHandoverAlgorithmType("ns3::NrA2A4RsrpHandoverAlgorithm");

    // Install NR devices
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Build a real IMSI -> Node lookup
    // std::map<uint64_t, Ptr<Node>> imsiToNode;
    // for (uint32_t i = 0; i < ueNetDev.GetN(); i++)
    // {
    //     Ptr<NrUeNetDevice> ueDev = ueNetDev.Get(i)->GetObject<NrUeNetDevice>();
    //     uint64_t realImsi = ueDev->GetImsi();
    //     imsiToNode[realImsi] = ueNodes.Get(i);
    //     std::cout << "UE index " << i << " -> real IMSI " << realImsi << std::endl;
    // }

    nrHelper->AssignStreams({.gnbDevs = gnbNetDev, .ueDevs = ueNetDev});

    for (uint32_t i = 0; i < gnbNodes.GetN(); i++)
    {
        // Set the transmission power for each gNB
        if (i!=total_gnb-1) NrHelper::GetGnbPhy(gnbNetDev.Get(i), 0)->SetTxPower(lbs); // legitimate BS: 48 dBm   
        else NrHelper::GetGnbPhy(gnbNetDev.Get(i), 0)->SetTxPower(fbs); // fake BS: 50 dBm
    }

    nrHelper->AddX2Interface(gnbNodes); 

    // Configure the FBS
    Ptr<NrGnbNetDevice> gnbDevice = gnbNetDev.Get(total_gnb - 1)->GetObject<NrGnbNetDevice>();
    Ptr<NrGnbRrc> gnbRrc = gnbDevice->GetRrc();
    gnbRrc->SetAttribute("AdmitHandoverRequest", BooleanValue(false));
    // gnbRrc->SetAttribute("AdmitRrcConnectionRequest", BooleanValue(false));

    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it) DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it) DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();

    // Create a remote host and connect it to the EPC's PGW
    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.010));

    // Add mobility for remoteHost here, right after it's created
    MobilityHelper remoteHostMobility;
    remoteHostMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    remoteHostMobility.Install(remoteHost);
    remoteHost->GetObject<MobilityModel>()->SetPosition(Vector(size_x + 200, 700.0, 0.0)); 

    // Install internet stack on UEs
    InternetStackHelper internet;
    internet.Install(ueNodes);

    // Assign IP addresses to UEs
    Ipv4InterfaceContainer ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    // Attach each UE to the closest legitimate gNB 
    double diff_x, diff_y, diff_z;

    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        Ptr<Node> ueNode = ueNodes.Get(i);

        Ptr<Node> closestGnbNode = nullptr;
        double minDistance = std::numeric_limits<double>::max();
        int closestGnbIndex = -1;

        for (uint32_t j = 0; j < gnbNodes.GetN()-1; j++)
        {
            Ptr<Node> gnbNode = gnbNodes.Get(j);

            Ptr<MobilityModel> ue_mobility = ueNode->GetObject<MobilityModel>();
            Ptr<MobilityModel> gnb_mobility = gnbNode->GetObject<MobilityModel>();

            if (ue_mobility && gnb_mobility)
            {
                Vector uePosition = ue_mobility->GetPosition();
                Vector gnbPosition = gnb_mobility->GetPosition();

                diff_x = std::pow(uePosition.x - gnbPosition.x, 2);
                diff_y = std::pow(uePosition.y - gnbPosition.y, 2);
                diff_z = std::pow(uePosition.z - gnbPosition.z, 2);

                double distance = std::sqrt(diff_x + diff_y + diff_z);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestGnbNode = gnbNode;
                    closestGnbIndex = j;
                }
            }
        }

        // Attach the UE to the closest legitimate gNB
        if (closestGnbNode)
        {
            nrHelper->AttachToGnb(ueNetDev.Get(i), gnbNetDev.Get(closestGnbIndex));
        }
    }

    if (enableAnimation)
    {
        anim = new AnimationInterface(xmlFileName);
        anim->SkipPacketTracing(); 
        anim->SetMobilityPollInterval(Seconds(2)); 

        for (uint8_t i=0;i<total_gnb;i++)
        {
            anim->UpdateNodeColor(gnbNodes.Get(i),std::get<0>(colors[i]),std::get<1>(colors[i]),std::get<2>(colors[i]));
            anim->UpdateNodeSize(gnbNodes.Get(i),50.0,50.0);
            anim->UpdateNodeDescription(gnbNodes.Get(i),std::to_string(i+1));
        }

        for (uint8_t i = 0; i < total_ue; i++)
        {
                uint16_t cellId = ueNodes.Get(i)->GetDevice(0)->GetObject<NrUeNetDevice>()->GetRrc()->GetCellId();

            anim->UpdateNodeColor(ueNodes.Get(i), std::get<0>(colors[cellId-1]), std::get<1>(colors[cellId-1]), std::get<2>(colors[cellId-1]));
            anim->UpdateNodeSize(ueNodes.Get(i), 15.0, 15.0);
            anim->UpdateNodeDescription(ueNodes.Get(i), "");
        }

        for (uint8_t i = 0;i < 4; i++)
        {
            anim->UpdateNodeColor(NodeList::GetNode(i+total_ue+total_gnb), 0, 255, 255);
            anim->UpdateNodeSize(NodeList::GetNode(i+total_ue+total_gnb), 50.0, 50.0);
            anim->UpdateNodeDescription(NodeList::GetNode(i+total_ue+total_gnb),"");
        }
    }

    AsciiTraceHelper asciiTraceHelper;
    Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream(txtFileName);
    *stream->GetStream() << "Time\tIMSI\tUE-X\tUE-Y\tUE-Z\tServingCellId\tNumOfNeighbour\
    \tRSRP1\tX1\tY1\tZ1\tDistance1\tUE-gNB1-Distance\
    \tRSRP2\tX2\tY2\tZ2\tDistance2\tUE-gNB2-Distance\
    \tRSRP3\tX3\tY3\tZ3\tDistance3\tUE-gNB3-Distance\
    \tRSRP4\tX4\tY4\tZ4\tDistance4\tUE-gNB4-Distance\
    \tRSRP5\tX5\tY5\tZ5\tDistance5\tUE-gNB5-Distance\
    \tRSRP6\tX6\tY6\tZ6\tDistance6\tUE-gNB6-Distance\
    \tRSRP7\tX7\tY7\tZ7\tDistance7\tUE-gNB7-Distance\
    \tRSRP8\tX8\tY8\tZ8\tDistance8\tUE-gNB8-Distance\
    \n";

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrNetDevice/$ns3::NrGnbNetDevice/NrGnbRrc/RecvMeasurementReport",
                 MakeBoundCallback(&RecvMeasurementReportCallback, stream));

    Simulator::Stop(Seconds(simTime));
    for (double i = 0; i < simTime; i+=1)
    {   
        if (enableAnimation) Simulator::Schedule(Seconds(i), &UpdateColors,ueNodes);  
        Simulator::Schedule(Seconds(i), &PrintProgress);
    }

    Simulator::Run();
    Simulator::Destroy();

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    return 0;
}