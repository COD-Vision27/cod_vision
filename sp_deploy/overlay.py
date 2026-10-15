"""Build-only IO overlay: expose fresh received quaternion without consuming frame history."""
import hashlib,shutil
from pathlib import Path

def once(text,old,new):
    if text.count(old)!=1:raise ValueError('SP IO source layout changed; overlay anchor is not unique')
    return text.replace(old,new)

def generate(source,output):
    source=Path(source);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    shutil.copytree(source/'io',output/'io',dirs_exist_ok=True)
    header=source/'io/gimbal/gimbal.hpp';cpp=source/'io/gimbal/gimbal.cpp'
    h=once(header.read_text(),'  GimbalCommunicationStats communication_stats() const;',
        '  GimbalCommunicationStats communication_stats() const;\n  std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> latest_pose() const;')
    h=once(h,'  GimbalCommunicationStats communication_stats_{};',
        '  GimbalCommunicationStats communication_stats_{};\n  Eigen::Quaterniond latest_q_{Eigen::Quaterniond::Identity()};\n  std::chrono::steady_clock::time_point latest_pose_time_{};')
    c=once(cpp.read_text(),'    auto & stats = communication_stats_;',
        '    latest_q_ = q;\n    latest_pose_time_ = t;\n    auto & stats = communication_stats_;')
    c=once(c,'void Gimbal::send(io::VisionToGimbal VisionToGimbal)',
        'std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> Gimbal::latest_pose() const\n{\n  std::lock_guard<std::mutex> lock(mutex_);\n  return {latest_q_, latest_pose_time_};\n}\n\nvoid Gimbal::send(io::VisionToGimbal VisionToGimbal)')
    (output/'io/gimbal/gimbal.hpp').write_text(h);(output/'io/gimbal/gimbal.cpp').write_text(c)
    return {'io/gimbal/gimbal.hpp':hashlib.sha256(header.read_bytes()).hexdigest(),
            'io/gimbal/gimbal.cpp':hashlib.sha256(cpp.read_bytes()).hexdigest()}
