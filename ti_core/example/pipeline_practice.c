/**
 * Pipeline a graph with two node
 *
 * In OpenVX, graph pipelining allows multiple instance of a graph to execute simulataneously each operating of different input and outout data.
 *
 * In this tutorial we learn the below concepts,
 *
 * - How to create OpenVX graph with two nodes and pipeline across two different target CPUs
 *
 */

#include <stdio.h>
#include <VX/vx_khr_pipelining.h>
#include <TI/tivx.h>
#include <VX/vxu.h>

/* max number of buffers to allocate for input and output */
#define MAX_NUM_BUF (2u)

/**
 * Utility API used to add a graph parameter from a node, node parameter index
 */
static void add_graph_parameter_by_node_index(vx_graph graph,
                                              vx_node node,
                                              vx_uint32 node_parameter_index)
{
    // vxGetParameterByIndex(): Retrieves a vx_parameter from a vx_node.
    // 所以 node_parameter_index 代表了 node 中的参数 idx
    // typedef struct _vx_parameter* vx_parameter
    // An opaque reference to a single parameter.
    vx_parameter parameter = vxGetParameterByIndex(node, node_parameter_index);
    // vxAddParameterToGraph(): Adds the given parameter extracted from a vx_node to the graph.
    vxAddParameterToGraph(graph, parameter);
    vxReleaseParameter(&parameter);
}

static void print_performance(vx_perf_t performance,
                              uint32_t numPixels,
                              const char *testName)
{
    printf("[ %c%c ] Execution time for %9d pixels (avg = %4.6f ms, min = %4.6f ms, max = %4.6f ms)\n",
           testName[0], testName[1],
           numPixels,
           performance.avg / 1000000.0,
           performance.min / 1000000.0,
           performance.max / 1000000.0);
}

static void print_graph_pipeline_performance(vx_graph graph,
                                             vx_node nodes[],
                                             uint32_t num_nodes,
                                             uint64_t exe_time,
                                             uint32_t loop_cnt,
                                             uint32_t numPixels)
{
    vx_perf_t perf_ref;
    char ref_name[8];
    uint32_t i;
    uint64_t avg_exe_time;
    avg_exe_time = exe_time / loop_cnt;
    for (i = 0; i < num_nodes; i++)
    {
        vxQueryNode(nodes[i], (vx_enum)VX_NODE_PERFORMANCE, &perf_ref, sizeof(perf_ref));
        snprintf(ref_name, 8, "N%d ", i);
        print_performance(perf_ref, numPixels, ref_name);
    }
    printf("[ SYS ] Execution time (avg = %4d.%03d ms, sum = %4d.%03d ms, num = %d)\n",
           (uint32_t)(avg_exe_time / 1000u), (uint32_t)(avg_exe_time % 1000u),
           (uint32_t)(exe_time / 1000u), (uint32_t)(exe_time % 1000u),
           loop_cnt);
}
void vx_tutorial_graph_pipeline_two_nodes()
{
    /**
     * - Define objects that we wish to create in the OpenVX application.
     *
     * - in_img[] are the input images that we will pass to the graph
     * - out_img[] are the output images we will get back from the graph
     * - tmp is the intermediate image, we will not access this within the application
     *
     * - n0 and n1 are the two NOT nodes running on two different target CPUs
     *
     * - graph_parameters_queue_params_list[] are the graph parameters info that
     *   we will pipeline and associate with the graph
     *
     */
    vx_context context;
    vx_image in_img[MAX_NUM_BUF], tmp, out_img[MAX_NUM_BUF];
    vx_node n0, n1;
    vx_graph_parameter_queue_params_t graph_parameters_queue_params_list[2];
    vx_graph graph;
    uint32_t width, height, num_buf, pipeline_depth, buf_id, loop_id, loop_cnt, exe_time;
    printf(" vx_tutorial_graph_pipeline_two_nodes: Tutorial Started !!! \n");
    context = vxCreateContext();
    width = 64;
    height = 64;
    num_buf = MAX_NUM_BUF;
    pipeline_depth = MAX_NUM_BUF;
    loop_cnt = 10 - num_buf; /* runs the graph 10 times */
    graph = vxCreateGraph(context);

    /**
     * - Allocate Input and Output images, multiple refs created to allow pipelining of graph.
     *
     * - Allocate intermediate image, application creates a single intermediate vx_image.
     *   It is converted to multiple vx_images internally using tivxSetNodeParameterNumBufByIndex()
     *   later.
     */
    for (buf_id = 0; buf_id < num_buf; buf_id++)
    {
        in_img[buf_id] = vxCreateImage(context, width, height, (vx_df_image)VX_DF_IMAGE_U8);
        out_img[buf_id] = vxCreateImage(context, width, height, (vx_df_image)VX_DF_IMAGE_U8);
    }
    tmp = vxCreateImage(context, width, height, (vx_df_image)VX_DF_IMAGE_U8);

    /**
     * - Contruct the graph and set node targets such that each runs
     *    on a different CPU target
     */
    n0 = vxNotNode(graph, in_img[0], tmp);
    vxSetNodeTarget(n0, (vx_enum)VX_TARGET_STRING, TIVX_TARGET_DSP1);

    n1 = vxNotNode(graph, tmp, out_img[0]);
    vxSetNodeTarget(n1, (vx_enum)VX_TARGET_STRING, TIVX_TARGET_DSP2);

    /**
     * The position where input and output images are made as graph
     *   parameters.
     *
     * This is because for pipelining, one can only enqueue/dequeue
     *   into a graph parameter
     *
     * 只有 graph parameters 用作 enqueue/dequeue
     * graph parameters 指代的不是 node， 而是 node 的输入或者输出
     *
     */
    /* input @ n0 index 0, becomes graph parameter 0 */
    // n0 节点的 index0 是 input image
    // 将它加到 graph parameters 中, 成为了 index0 of graph parameters
    add_graph_parameter_by_node_index(graph, n0, 0);
    /* output @ n1 index 1, becomes graph parameter 1 */
    // n1 节点的 index1 是 output image
    // 将它加到 graph parameters 中, 成为了 index1 of graph parameters
    add_graph_parameter_by_node_index(graph, n1, 1);

    /**
     * - Set graph scehdule policy to make it pipelined.
     *
     *   - This is done by providing a list of graph parameters that user wants to
     *     enqueue or dequeue. This is the input and output graph parameters in this case.
     *
     *   - The number of buffers that can be enqueued before its internal queue becomes
     *     full is also specified via graph_parameters_queue_params_list[0].refs_list_size.
     *
     *   - The list of buffers that could be every enqueued is specified via,
     *     graph_parameters_queue_params_list[0].refs_list
     */
    /* set graph schedule config such that graph parameter @ index 0 and 1 are enqueuable */
    // 就是说如果只是用过 add_graph_parameter_by_node_index() 加入了 graph parameters 还不够
    // 必须明确指明 buffers 数量以及 buffers 的引用列表
    // refs_list 就是 buffers 的引用列表, 上面创建时, 就已经为 in_img 分配了 2 个 buffer，分配的大小与 refs_list_size 应该是一致的
    // 在这里只是把这个已经声明好的数组首地址转换成(vx_reference *)类型，加入到 graph_parameters_queue_params_list 中
    // 
    // refs_list 的定义描述为：Array of references that could be enqueued at a later point of time at this graph parameter.
    // 就是说 refs_list 代表了在这个 graph parameter index 的位置上，可以 enqueue 的 array of reference，
    // 所以如果之后 enqueue 了相同类型但是不是同一个 data object 的 reference 是不可以的，
    // 可以参考下面 vxGraphParameterEnqueueReadyRef() 的注释内容
    // 另外，refs_list 中每个元素的 meta_data 都应该与其余元素的 meta_data 相同，开发者要自行保证这一点，framework 渣渣检查不出来
    graph_parameters_queue_params_list[0].graph_parameter_index = 0;
    graph_parameters_queue_params_list[0].refs_list_size = num_buf;
    graph_parameters_queue_params_list[0].refs_list = (vx_reference *)&in_img[0];

    graph_parameters_queue_params_list[1].graph_parameter_index = 1;
    graph_parameters_queue_params_list[1].refs_list_size = num_buf;
    graph_parameters_queue_params_list[1].refs_list = (vx_reference *)&out_img[0];

    /* 
     * Schedule mode auto is used, here we dont need to call vxScheduleGraph
     * Graph gets scheduled automatically as refs are enqueued to it
     * 
     * This method is used for setting the graph scheduler config to allow user to schedule multiple instances of a graph for execution.
     * 
     * For legacy applications that don't need graph pipelining or batch processing, this API need not be used.
     * 只有使用 graph pipelining 或者 batch processing 时，才需要 schedule graph config
     * 
     * A single monolithic API is provided instead of discrete APIs, 
     * since this allows the implementation to get all information related to scheduling in one shot and then optimize the subsequent graph scheduling based on this information.
     * 
     * This API MUST be called before graph verify since in this case it allows implementations the opportunity to optimize resources based on information provided by the application.
     * 在 graph verify 之前调用此 API，这样后续才能利用这些信息去优化程序资源
     * 
     * 'graph_schedule_mode' selects how input and output references are provided to a graph and how the next graph schedule is triggered by an implementation.
     * 
     * VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO: 
     *          
     *          + Application needs to explicitly call vxVerifyGraph() before enqueing data references
     *              显式调用 vxVerifyGraph()，而且在 enqueue 之前
     *          
     *          + Application should not call vxScheduleGraph() or vxProcessGraph()
     *          
     *          + When enough references are enqueued at various graph parameters, the implementation could trigger the next graph schedule.
     *              文档里说，如果没有足够的 buffer 来保存结果，graph 就会 stall；换句话说，就是只要 graph 中 buffer 是足够的 graph 就会一直运行下去
     *              stall 时，graph 会停止运行，dequeue/enqueue，直到 graph parameters 中的每一个 buffer 都足够。
     *              并不是之前理解的只有 index0 的 graph parameters enqueue 之后才继续运行 
     * 
     * VX_GRAPH_SCHEDULE_MODE_QUEUE_MANUAL:
     * 
     *          + Application needs to explicitly call vxScheduleGraph(). Application should not call vxProcessGraph()
     *              显式调用 vxScheduleGraph() 才能开始执行 graph， 调用 vxProcessGraph() 不好使
     * 
     *          + 因为是 Manual mode，所以必须手动调用 vxScheduleGraph() trigger graph execution；另外，在调用 vxScheduleGraph() 之前，所有的 graph parameter 都必须 enqueue 完毕
     *          
     *          + User can use vxWaitGraph() to wait for the previous vxScheduleGraph() to complete.
     *              vxWaitGraph() 类似一个同步指令
     *          
     *          + Application can enqueue multiple references at the same graph parameter. When vxScheduleGraph() is called, all enqueued references get processed in a 'batch'.
     *              在同一个 graph parameter 上 enqueue 多次，实现 batch 操作？
     * 
     *  VX_GRAPH_SCHEDULE_MODE_NORMAL:
     * 
     *          + 'graph_parameters_list_size' MUST be 0 and 'graph_parameters_queue_params_list' MUST be NULL
     *          
     *          + This mode is equivalent to non-queueing scheduling mode as defined by OpenVX v1.2 and earlier.
     * 
     *          + By default all graphs are in VX_GRAPH_SCHEDULE_MODE_NORMAL mode until vxSetGraphScheduleConfig() API is called.
     *              不会使用这个模式的，使用这个模式是不可能了，这辈子都不会使用这个模式
     *              
     * 
     * 'graph_parameters_queue_params_list'： 指定哪些 graph parameter 可以 enqueue，并传入 enqueue 变量的引用列表
     * 
     *          + Application MUST use vxGraphParameterEnqueueReadyRef() to set references at the graph parameter. 
     *            Using other data access API's on these parameters or corresponding data objects will return an error.
     *              只有那些在 graph_parameters_queue_params_list 中的 graph parameter 才能支持 enqueue 操作
     *              enqueue 操作必须也只能通过 vxGraphParameterEnqueueReadyRef() 完成
     * 
     *          + For graph parameters not listed in 'graph_parameters_queue_params_list', 
     *            application MUST use the vxSetGraphParameterByIndex() to set the reference at the graph parameter. 
     *            Using other data access API's on these parameters or corresponding data objects will return an error.
     *              有一些 data object 添加到了 graph 中， 成为了 graph parameters，但是没有出现在 graph_parameters_queue_params_list 中
     *              对于这些 data objects，需要调用 vxSetGraphParameterByIndex() 手动设置 reference
     *              这个函数设置的是 reference，而不是 reference 数组，功能应该没有 graph_parameters_queue_params_list 强大
     * 
     * 
     * 在调用 vxVerifyGraph() 之前， 如果 reference handle 还没确定， refs_list 可以是 NULL，但是 refs_list_size 必须确定
     * 在 vxVerifyGraph() 之后， graph enqueue 之前， 可以再次调用 vxSetGraphScheduleConfig() 确定 refs_list 的内容
     * 
     */
    vxSetGraphScheduleConfig(graph,
                             (vx_enum)VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                             2, // graph_parameters_list_size
                             graph_parameters_queue_params_list);

    /**
     * - Set graph pipeline depth. This has to be done explicitly. Default is 1.
     */
    /* explicitly set graph pipeline depth */
    tivxSetGraphPipelineDepth(graph, pipeline_depth);

    /**
     * - Set number of intermediate buffers. This has to be done explicitly. Default is 1.
     */
    /* make the 'tmp' reference which is output of n0 @ index 1 a "multi-buffer" */
    /**
     * the second parameter is Node parameter index
     * the node parameter which is represented by the index must be a output parameter
     * the node parameter which is represented by the index must NOT be an enqueueable graph parameter
     *
     */
    tivxSetNodeParameterNumBufByIndex(n0, 1, num_buf);

    /**
     * - Verify the graph
     * - Optionally export the graph as graphviz dot graph to visualize the graph and
     *   its pipelined structure
     * - Optionally enable real-time logging to get a waveform like trace of graph execution
     */
    vxVerifyGraph(graph);
#if 0
    tivxExportGraphToDot(graph, ".", "vx_tutorial_graph_pipeline_two_nodes");
    tivxLogRtTrace(graph);
#endif
    exe_time = tivxPlatformGetTimeInUsecs();

    /**
     * - Enqueue input and output references,
     * - input and output can be enqueued in any order
     * - The moment something is enqueued at graph parameter 0, the graph execution begins
     */
    /**
     * vxGraphParameterEnqueueReadyRef(graph, graph_parameter_index, *refs, num_refs)
     *
     * Enqueues new references into a graph parameter for processing. This new reference will take effect on the next graph schedule.
     *
     * In case of a graph parameter which is input to a graph,
     *      this function provides a data reference with new input data to the graph.
     * In case of a graph parameter which is not input to a graph,
     *      this function provides a 'empty' reference into which a graph execution can write new data into.
     *
     * This function essentially transfers ownership of the reference from the application to the graph.
     * User MUST use vxGraphParameterDequeueDoneRef() to get back the processed or consumed references.
     *
     * The references that are enqueued MUST be the references listed during vxSetGraphScheduleConfig().
     * If a reference outside this list is provided then behaviour is undefined.
     * 
     * 有一事不明，num_refs 是做什么的？
     * 是在 VX_GRAPH_SCHEDULE_MODE_QUEUE_MANUAL 部分谈到的 batch 操作吗？
     *
     */
    for (buf_id = 0; buf_id < num_buf; buf_id++)
    {
        /* reset output */
        vxGraphParameterEnqueueReadyRef(graph, 1, (vx_reference *)&out_img[buf_id], 1);
        /* load input */
        vxGraphParameterEnqueueReadyRef(graph, 0, (vx_reference *)&in_img[buf_id], 1);
    }

    /**
     * - Wait for graph instances to complete,
     * - Compare output and recycle data buffers, schedule again
     */
    // width = 64;
    // height = 64;
    // num_buf = MAX_NUM_BUF;
    // pipeline_depth = MAX_NUM_BUF;
    // loop_cnt = 10 - num_buf; /* runs the graph 10 times */
    for (buf_id = loop_id = 0; loop_id < (loop_cnt + num_buf); loop_id++)
    {
        vx_image cur_out_img, cur_in_img;
        uint32_t num_refs;
        /* Get output reference, waits until a reference is available */
        /**
         * vxGraphParameterDequeueDoneRef(graph, graph_parameter_index, *refs, max_refs, num_refs)
         *
         * Dequeues 'consumed' references from a graph parameter.
         *
         * This function dequeues references from a graph parameter of a graph.
         * The reference that is dequeued is a reference that had been previously enqueued into a graph,
         * and after subsequent graph execution is considered as processed or consumed by the graph.
         *
         * This function essentially transfers ownership of the reference from the graph to the application.
         *
         * IMPORTANT : This API will block until at least one reference is dequeued.
         * API will block app until a reference is dequeued
         *
         * In case of a graph parameter which is input to a graph,
         *      this function provides a 'consumed' buffer to the application so that new input data can filled and later enqueued to the graph.
         * In case of a graph parameter which is not input to a graph,
         *      this function provides a reference filled with new data based on graph execution.
         *
         * User can then use this newly generated data with their application.
         * Typically when this new data is consumed by the application the 'empty' reference is again enqueued to the graph.
         *
         * This API returns an array of references up to a maximum of 'max_refs'.
         * Application MUST ensure the array pointer ('refs') passed as input can hold 'max_refs'.
         * 'num_refs' is actual number of references returned and will be <= 'max_refs'.
         * 
         * 可能一次返回多个 reference， 通过 num_refs 来判断具体返回了多少个； num_refs 不会大于 max_refs
         * max_refs 是倒数第二个参数， 在这个例子里是 1
         * 
         * 关于 Enqueue 与 Dequeue，有一点不太清楚，这里传入的 reference array pointer，只是 1级指针
         * 所以数组每个元素可能在函数中被修改，那么 dequeue 出来的内容是拷贝过来的，还是说这部分内存在 target 上也能使用
         * target 上的结果就直接写在了上面
         * 根据文档的描述，enqueue 与 dequeue 只是转换了 ownership of the reference
         * 再加上之前调试时间戳 bug 时的现象， 我估计 refernce 指向的内存属于一种共享内存， target 的结果直接写在了上面
         * 不过也有可能是在 target 上运行之后，进行了一次内存拷贝，拷贝到了这个共享内存上
         *
         */
        vxGraphParameterDequeueDoneRef(graph, 1, (vx_reference *)&cur_out_img, 1, &num_refs);
        /* Get consumed input reference, waits until a reference is available
         */
        vxGraphParameterDequeueDoneRef(graph, 0, (vx_reference *)&cur_in_img, 1, &num_refs);
        /* A graph execution completed, since we dequeued both input and output refs */
        /* use output */
        buf_id = (buf_id + 1) % num_buf;
        /* recycles dequeued input and output refs 'loop_cnt' times */
        if (loop_id < loop_cnt)
        {
            /* input and output can be enqueued in any order */
            vxGraphParameterEnqueueReadyRef(graph, 1, (vx_reference *)&cur_out_img, 1);
            vxGraphParameterEnqueueReadyRef(graph, 0, (vx_reference *)&cur_in_img, 1);
        }
    }

    /* ensure all graph processing is complete */
    vxWaitGraph(graph);
    exe_time = tivxPlatformGetTimeInUsecs() - exe_time;
    {
        vx_node nodes[] = {n0, n1};
        print_graph_pipeline_performance(graph, nodes, 2, exe_time, loop_cnt + num_buf, width * height);
    }
    /**
     * - Release all perviously allocated resources
     */
    vxReleaseNode(&n0);
    vxReleaseNode(&n1);
    for (buf_id = 0; buf_id < num_buf; buf_id++)
    {
        vxReleaseImage(&in_img[buf_id]);
        vxReleaseImage(&out_img[buf_id]);
    }
    vxReleaseImage(&tmp);
    vxReleaseGraph(&graph);
    vxReleaseContext(&context);
    printf(" vx_tutorial_graph_pipeline_two_nodes: Tutorial Done !!! \n");
}
