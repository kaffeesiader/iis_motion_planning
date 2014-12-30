function writeTriMesh(T,TN,V,VN, filename)


[N,d] = size(T);

if d~=3
    error('Bad parameters (T)');
end

[d,NV] = size(V);

if d~=3
    error('Bad parameters (V)');
end

if ~isempty(TN)
    [N2,d2] = size(TN);

    if d2~=3 | N~=N2
        error('Bad parameters, TN');
    end

    [d2,NVN] = size(VN);

    if d2~=3 
        error('Bad parameters, VN');
    end
end

fid = fopen(filename,'w');

if fid==-1
   error('File cannot be found or opened for writing.');
end

fprintf(fid,'TRI\n%i\n%i\n\n',NV,N);    

for i=1:NV
    fprintf(fid,' %f %f %f\n',V(:,i));
end
fprintf(fid,'\n');
if ~isempty(TN)
    for i=1:N
        fprintf(fid,' %i %i %i\n', T(i,1)-1, T(i,2)-1, TN(i,3)-1);
    end

else
    for i=1:N
        fprintf(fid,' %i %i %i\n', T(i,1)-1, T(i,2)-1, T(i,3)-1);
    end
end    

fclose(fid);
