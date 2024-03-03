#include <string>
#include <fstream>
#include <sstream>
#include "base/map.h"
#include "3rdparty/json/json.hpp"
#include "mapper/incremental_mapper.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"

using namespace xrsfm;

struct KeyFrameVIOCsv {
    struct KeyFrameVIOData {
        std::string imagename;
        double timestamp;
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
    };

    std::vector<KeyFrameVIOData> items;
    void load(const std::string &filename) {
        std::ifstream fs;
        fs.open(filename.c_str());
        std::string line;
        std::istringstream sin;
        KeyFrameVIOData item;
        while (std::getline(fs, line) && !line.empty()) {
            sin.clear();
            sin.str(line);
            sin >> item.imagename  >> item.timestamp >> item.t.x() >> item.t.y() >> item.t.z() 
                >> item.q.w() >> item.q.x() >> item.q.y() >> item.q.z();
            // std::cout << item.imagename  << " " << item.timestamp << " " << item.q.z() << std::endl;
            items.emplace_back(std::move(item));
        }
        fs.close();
    }
};

void PreProcess(const std::string dir_path, Map &map) {
    std::vector<Frame> frames;
    std::vector<FramePair> frame_pairs;
    ReadFeatures(dir_path + "ftr.bin", frames, true);
    ReadFramePairs(dir_path + "fp.bin", frame_pairs);

    // set cameras & image name
    Camera seq;
    seq = Camera(0, 461.158, 459.752, 362.659, 248.521, 0.0);

    std::string images_path = dir_path + "KeyFrameInfo.txt";
    KeyFrameVIOCsv VIOCsv;
    VIOCsv.load(images_path);

    size_t i = 0;
    // convert keypoint to points(for reconstruction)
    for (auto &frame : frames) {
        frame.timestamp = VIOCsv.items[i].timestamp;
        
        Eigen::Matrix3d Rwc;
        Rwc = VIOCsv.items[i].q.toRotationMatrix();
        Eigen::Matrix3d Rcw = Rwc.transpose();
        Eigen::Vector3d t = -1.0  * Rcw * VIOCsv.items[i].t;
        frame.Tcw = Pose(quaternion(Rcw), t);
        frame.camera_id = seq.id_;
        const int num_points = frame.keypoints_.size();
        frame.points.clear();
        frame.track_ids_.assign(num_points, -1);
        for (const auto &kpt : frame.keypoints_) {
            const auto &pt = kpt.pt;
            Eigen::Vector2d ept(pt.x, pt.y), eptn;
            frame.points.push_back(ept);
        }
        i++;
    }
    map.camera_map_[seq.id_] = seq;
    map.frames_ = frames;
    map.frame_pairs_ = frame_pairs;

    map.RemoveRedundancyPoints();
    map.Init();
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    // 1.Read Config
    std::string bin_path, output_path;
    int init_id1 = -1, init_id2 = -1;
    if (argc <= 2) {
        std::string config_path = "./config_seq.json";
        if (argc == 2) {
            config_path = argv[1];
        }
        auto config_json = LoadJSON(config_path);
        bin_path = config_json["bin_path"];
        output_path = config_json["output_path"];
        init_id1 = config_json["init_id1"];
        init_id2 = config_json["init_id2"];
    } else if (argc >= 3 && argc <= 5) {
        bin_path = argv[1];
        output_path = argv[2];
        if (argc >= 4) {
            init_id1 = std::stoi(argv[3]);
        }
        if (argc == 5) {
            init_id2 = std::stoi(argv[4]);
        }
    } else {
        exit(-1);
    }
    std::cout << "Read Config Done!" << std::endl;

    // 2. Map PreProcess
    Map map;
    PreProcess(bin_path, map);
    std::cout << "PreProcess Done!" << std::endl;

    // 3. Map Reconstruction
    IncrementalMapper imapper;
    imapper.options.init_id1 = init_id1;
    imapper.options.init_id2 = init_id2;
    imapper.options.correct_pose = false;
    imapper.options.stop_when_register_fail = true;
    imapper.Reconstruct(map);
    std::cout << "Reconstruction Done!" << std::endl;

    // 4. Output
    WriteColMapDataBinary(output_path, map);

    return 0;
}
