function twin_log_write(log_file, epoch, y_meas, y_pred, innov, u_mpc, triggered, y_nompc)
%TWIN_LOG_WRITE  Append one row to the twin log CSV.
%
%   Columns: epoch, y1_meas, y2_meas, y3_meas, y1_pred, y2_pred, y3_pred,
%            innov1, innov2, innov3, u_mpc1, u_mpc2, u_mpc3, triggered,
%            y1_nompc, y2_nompc, y3_nompc

header = 'epoch,y1_meas,y2_meas,y3_meas,y1_pred,y2_pred,y3_pred,innov1,innov2,innov3,u_mpc1,u_mpc2,u_mpc3,triggered,y1_nompc,y2_nompc,y3_nompc';

if nargin < 8 || isempty(y_nompc)
    y_nompc = nan(3,1);
end

write_header = ~isfile(log_file);

parent = fileparts(log_file);
if ~isempty(parent) && ~isfolder(parent)
    error('twin_log_write: directory does not exist: %s', parent);
end

fid = fopen(log_file, 'a');
if fid == -1
    error('twin_log_write: cannot open %s', log_file);
end
try
    if write_header
        fprintf(fid, '%s\n', header);
    end
    row = [epoch, y_meas(:)', y_pred(:)', innov(:)', u_mpc(:)', triggered, y_nompc(:)'];
    fprintf(fid, '%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%g,%g,%g,%d,%.6f,%.6f,%.6f\n', row);
finally
    fclose(fid);
end
end
