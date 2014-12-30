function [T, TN, V, VN] = loadObjMesh(filename)

fid = fopen(filename,'r');

if fid==-1
   error('File cannot be found or opened for reading.');
end

V = [];
VN = [];
F = [];
TN = [];

nr = 1;
while ~feof(fid)
   s = fgetl(fid);
   nr = nr+1;
   
   if isempty(s)
      continue
   end
   
   switch(s(1))
      case 'v'
         v = sscanf(s(2:end),'%f');
         if length(v)==3
            V=[V v];
         elseif s(2)=='n'
            vn = sscanf(s(3:end),'%f');
            if length(vn)==3
               VN = [VN vn];
            end
         end
      case 'f'
         t = sscanf(s(2:end),'%i//%i');
         switch length(t)
            case 3
               F = [F; t'];
            case 6
               F = [F; t(1) t(3) t(5)];
               TN = [TN; t(2) t(4) t(6)];
                  
            case 4
               F = [F; t(1) t(2) t(3); t(3) t(4) t(1)];
            case 8
               F = [F; t(1) t(3) t(5); t(5) t(7) t(1)];
            otherwise
               t = sscanf(s(2:end),'%i');
               if length(t)==3
                  F = [F; t'];
               end
         end
   end
end

fclose(fid);

T=F;
