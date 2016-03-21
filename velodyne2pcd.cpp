#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "readKitti.h"
using namespace std;

namespace po = boost::program_options;
namespace fs = boost::filesystem;
std::string output_path;
bool save_as_ascii = false;
struct ScanFileComp{
    bool operator()(const fs::path& p1, const fs::path& p2){
        return p1.stem().string() < p2.stem().string();
    }
};

void saveAsPCD(const CloudXYZI& cloud, const fs::path& fullpath){
    if(save_as_ascii)
        pcl::io::savePCDFileASCII(fullpath.string(), cloud);
    else
        pcl::io::savePCDFileBinary(fullpath.string(), cloud);

}

int getFilesList(const std::string& path_name, vector<fs::path>& files_list){
    fs::path path(path_name);

    if(!fs::is_directory(path)) {
        std::cerr<<path.c_str()<<" is not a directory"<<std::endl;
        return 0;
    }

    files_list.clear();
    for(fs::directory_iterator file(path), f_end; file!= f_end; ++file){
        files_list.push_back(file->path());
    }

    std::sort(files_list.begin(), files_list.end(), ScanFileComp());
    cout<<"Total "<< files_list.size()<<" files are found"<<endl;
    return 1;
}


int main(int argc, char** argv) {
    string single_file;
    vector<string> files;
    string folder;

    po::options_description desc("Kitti Velodyne Scans to PCD files", 120);
    desc.add_options()
            ("ascii,a", po::bool_switch(&save_as_ascii), "save pcd as ASCII code. Default : binary. e.g. -a")
            ("file,s", po::value<string>(&single_file), "single file. e.g. --file 000001.bin")
            ("folder,d", po::value<string>(&folder), "entire folder. e.g. --folder sequences/00/velodyne")
            ("files,m", po::value<vector<string> >(&files)->multitoken(),
             "multiple files. e.g. --files 000001.bin 000002.bin")
            ("outpath,o", po::value<string>(&output_path)->default_value(fs::current_path().string()),
             "path to where pcd files are saved. Default is current directory. e.g. -o output");
    po::variables_map vm;

    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        int tmp = vm.count("file") + vm.count("folder") + vm.count("files");

        if (tmp==0 || vm.count("help")) {
            cout << desc << endl;
            return 0;
        }

        if (tmp != 1) {
            cout << "Only one option is allowed at once." << endl;
            cout << desc << endl;
            return 0;
        }

        po::notify(vm);

        vector<fs::path> files_to_be_converted;
        if (vm.count("file")) {
            cerr << single_file << endl;
            files_to_be_converted.push_back(fs::path(single_file));
        }
        if (vm.count("folder")) {
            cerr << folder << endl;
            if (!getFilesList(folder, files_to_be_converted))
                return 0;
        }
        if (vm.count("files")) {
            for (vector<string>::const_iterator iter = files.begin(); iter != files.end(); iter++) {
                files_to_be_converted.push_back(fs::path(*iter));
            }
        }

        fs::path o_path(output_path);
        o_path /= fs::path("pcds");

        fs::create_directory(o_path);
        cout << "Saving pcd files to " << o_path << endl;
        for (vector<fs::path>::const_iterator iter = files_to_be_converted.begin();
             iter != files_to_be_converted.end(); iter++) {
            CloudXYZI cloud;
            readKittiVelodyne(*iter, cloud);
            saveAsPCD(cloud, o_path / iter->filename().replace_extension("pcd"));
        }
    }catch (std::exception& e){
        cerr<<e.what()<<endl;
        cout << desc << endl;
    }

    return 0;
}