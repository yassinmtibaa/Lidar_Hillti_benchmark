% Run both LOAM and LeGO-LOAM on an example bag (adjust path/topic)
bag = "data/site3_handheld_1.bag";
topic = "/hesai/pandar";

try
    algorithms.loam.pipeline(bag, topic);
catch ME
    warning("LOAM failed: %s", ME.message);
end

try
    algorithms.legoloam.pipeline(bag, topic);
catch ME
    warning("LeGO-LOAM failed: %s", ME.message);
end

