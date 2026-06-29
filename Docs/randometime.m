function meas_vec = randometime(time_vector, Tperiod, Tvariance, Toffset)
    %loop thrugh time vector and set 1 at measuring points intervals
    meas_vec = [];
    ref_vec = Toffset:Tperiod:time_vector(end);
    n_ref = 1;
    if(Tperiod<Tvariance)
        meas_vec = [1];
        return
    end
    for i = 1:numel(time_vector)
        if((time_vector(i) > ref_vec(n_ref))&&(n_ref < numel(ref_vec)))
            meas_vec(i) = 1;
            n_ref = n_ref +1;
        else
            meas_vec(i) = 0;
        end
        
    end
end