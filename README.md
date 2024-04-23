# 1

解决了建图模块不建图的问题

修改部分：

        if(mbuseExact)
	{
		syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor,*subImageDepth, *tcw_sub, *path_sub);
		syncExact->registerCallback(boost::bind(&PointCloudMapper::callback, this, _1, _2, _3, _4));
	}
	else
	{

		syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor,*subImageDepth, *tcw_sub, *path_sub);
		syncApproximate->registerCallback(boost::bind(&PointCloudMapper::callback, this, _1, _2, _3, _4));
	}//使用的

 在jetson nx中设置queueSize==10，能够保证建图模块的更新速度正常，否则建图模块无法正常更新
