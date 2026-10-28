from pathlib import Path
import copy, importlib.util, json, time
import numpy as np
import torch
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('detector',root/'src/rslidar_sdk/3D_detector.py')
module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
logger=common_utils.create_logger()
for port in (6636,6639,6641):
    config=copy.deepcopy(cfg)
    cfg_from_yaml_file(str(root/f'src/rslidar_sdk/config/pv_rcnn{port}.yaml'),config)
    dataset=module.RosDemoDataset(config.DATA_CONFIG,config.CLASS_NAMES,False,logger)
    model=build_network(config.MODEL,len(config.CLASS_NAMES),dataset)
    model.load_params_from_file(str(root/f'src/rslidar_sdk/model/checkpoint_epoch_90_{port}.pth'),logger,to_cpu=True)
    model.cuda().eval()
    rng=np.random.default_rng(82)
    extent=np.array(config.DATA_CONFIG.POINT_CLOUD_RANGE)
    points=np.empty((20000,4),dtype=np.float32)
    points[:,:3]=rng.uniform(extent[:3]+.2,extent[3:]-.2,size=(len(points),3))
    points[:,3]=rng.uniform(0,255,len(points))
    batch=dataset.collate_batch([dataset.prepare_ros_data(points)])
    load_data_to_gpu(batch)
    start=time.monotonic()
    with torch.no_grad():
        predictions,_=model(batch)
    torch.cuda.synchronize()
    print(json.dumps({'synthetic_test':True,'port':port,'points':len(points),'predictions':len(predictions[0]['pred_boxes']),'seconds':time.monotonic()-start}),flush=True)
    del model,batch,predictions,dataset
    torch.cuda.empty_cache()
