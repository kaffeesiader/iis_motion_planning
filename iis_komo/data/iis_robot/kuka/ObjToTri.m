Files={'kuka_base_lo','kuka_cap','kuka_last_lo','kuka_link1_lo','kuka_link2_lo','kuka_link3_lo','kuka_link4_lo','kuka_link5_lo','kuka_mid','kuka_ring','kuka_wrist_lo'};

for i=1:size(Files,2)
    FileIn=[Files{i},'.obj'];
    FileOut=[Files{i},'.tri'];
    [T, TN, V, VN] = loadObjMesh(FileIn);
    writeTriMesh(T,TN,V,VN, FileOut);
    display(['Writing ',FileOut]);
end