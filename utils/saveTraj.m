function saveTraj(fname,traj)
[fpath,~,~] = fileparts(fname); if ~isfolder(fpath), mkdir(fpath); end
writematrix(traj,fname,"Delimiter"," ");
end 