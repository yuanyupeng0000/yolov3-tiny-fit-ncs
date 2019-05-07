#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include "common.h"
#include <sys/time.h>
int  get_random_port()//7001-8000
{
    struct timeval tpstart;

    gettimeofday(&tpstart,NULL);

    srand(tpstart.tv_usec);

    return (7000+1+(int) (1000.0*rand()/(RAND_MAX+1.0)));
//
//	srand(time(0));
//	printf("%d\n",rand()%100+1);
}
int create_joinable_thread(THREAD_ENTITY callback, int level, void *data)
{
    pthread_t tid;
    pthread_attr_t      attr;
    struct sched_param  schedParam;

    #if 1
    /* Initialize the thread attributes */
    if (pthread_attr_init(&attr)) {
        //__D("Failed to initialize thread attrs\n");
        return -1;
    }
    if (pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE))
    {
        //__D("Failed to set PTHREAD_CREATE_DETACH\n");
        return -1;
    }
    /* Force the thread to use custom scheduling attributes */
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) {
        //__D("Failed to set schedule inheritance attribute\n");
        return -1;
    }

//    /* Set the thread to be fifo real time scheduled */
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO)) {
        //__D("Failed to set FIFO scheduling policy\n");
        return -1;
    }
    schedParam.sched_priority = sched_get_priority_max(SCHED_FIFO)-level;
    if (pthread_attr_setschedparam(&attr, &schedParam)) {
        //__D("Failed to set scheduler parameters\n");
        return -1;
    }
    #endif
    //if(pthread_create(&tid,&attr,callback, data))
    if(pthread_create(&tid,NULL,callback, data))
    {
        return -1;
    }
    return tid;
}
int create_detach_thread(THREAD_ENTITY callback, int level, void *data)
{
    pthread_t tid;
    pthread_attr_t      attr;
    struct sched_param  schedParam;

    #if 1
    /* Initialize the thread attributes */
    if (pthread_attr_init(&attr)) {
        //__D("Failed to initialize thread attrs\n");
        return -1;
    }
    if (pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED))
    {
        //__D("Failed to set PTHREAD_CREATE_DETACH\n");
        return -1;
    }
    /* Force the thread to use custom scheduling attributes */
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) {
        //__D("Failed to set schedule inheritance attribute\n");
        return -1;
    }

//    /* Set the thread to be fifo real time scheduled */
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO)) {
        //__D("Failed to set FIFO scheduling policy\n");
        return -1;
    }
    schedParam.sched_priority = sched_get_priority_max(SCHED_FIFO)-level;
    if (pthread_attr_setschedparam(&attr, &schedParam)) {
        //__D("Failed to set scheduler parameters\n");
        return -1;
    }
    #endif
    //if(pthread_create(&tid,&attr,callback, data))
    if(pthread_create(&tid,NULL,callback, data))
    {
        return -1;
    }
    return tid;
}

void sig_handle(int signo,   siginfo_t *info,void* p)
{
    prt(err,"get signal %d",signo);
    if (signo == SIGSEGV) {
        exit(1);
    }
    if (signo == SIGPIPE) {
        prt(err,"ignore sig pipe");
    }
}
void watch_sig(int signo,void (*SignalAction)(int,siginfo_t*,void*))
{
    #if 1
    struct sigaction act;
    sigemptyset(&act.sa_mask);

    act.sa_flags=SA_SIGINFO;
    act.sa_sigaction = SignalAction;
    sigaction(signo, &act, NULL);
    #endif
}
void init_sig()
{
    watch_sig(SIGSEGV, sig_handle);//11
    watch_sig(SIGALRM, sig_handle);//14
    watch_sig(SIGTERM, sig_handle);//15
    watch_sig(SIGPWR , sig_handle);//30
    watch_sig(SIGKILL , sig_handle);
    watch_sig(SIGPIPE , sig_handle);//30
}

void *timed_fun(void *data)
{
    m_timed_func_data *p_data = (m_timed_func_data *) data;
    while (1) {
        usleep(p_data->time);
        p_data->func(p_data->data);
        pthread_testcancel();
    }
}
pthread_t regist_timed_callback(m_timed_func_data *p_ctx)
{
    return (pthread_t)create_detach_thread(timed_fun,1,p_ctx);
}
void unregist_timed_callback(pthread_t p)
{
    pthread_cancel(p);
}

m_timed_func_data *regist_timed_func(int time_us,void *ptr,void *data)
{
    m_timed_func_data *p_d=(m_timed_func_data *)malloc(sizeof(m_timed_func_data));
    p_d->data=data;
    p_d->func=(THREAD_ENTITY )ptr;
    p_d->time=time_us;
    return p_d;
}

pthread_t start_timed_func(m_timed_func_data *p_ctx)
{
    p_ctx->handle=(pthread_t)create_detach_thread(timed_fun,1,p_ctx);
    return p_ctx->handle;
}
pthread_t start_detached_func(void *func,void* data)
{
    return (pthread_t)create_detach_thread((THREAD_ENTITY)func,1,data);
}
pthread_t start_delayed_func(void *func,void* data,int delay_ms)
{
    usleep(delay_ms);
    return (pthread_t)create_detach_thread((THREAD_ENTITY)func,1,data);
}
void stop_timed_func(m_timed_func_data *p_ctx)
{

    pthread_cancel(p_ctx->handle);
    free(p_ctx);
}

m_list_info *new_list(int data_size,void *func)
{
    m_list_info *info_p=(m_list_info *)malloc(sizeof(m_list_info));
    info_p->head=NULL;
    info_p->cur=NULL;
    info_p->tail=NULL;
    info_p->number=0;
    info_p->data_size=data_size;
    info_p->data_match_function=(p_func)func;
    pthread_mutex_init(&info_p->list_lock,NULL);
    return info_p;
}
void delete_list(m_list_info *p_info)
{

    while(p_info->number!=0){
        //list_node_free_tail(p_info);
        m_node *tmp=p_info->tail;
        if(p_info->head!=NULL){
            if (p_info->tail->pre != NULL) {
                p_info->tail = p_info->tail->pre;
                p_info->tail->next = NULL;
            }
            free(tmp->data);
            free(tmp);
            p_info->number--;
        }else{
        }
    }
    free(p_info);
    pthread_mutex_destroy(&p_info->list_lock);
}
/* Function Description
 * name:
 * return:
 * args:
 * comment:
 * todo:
 */
extern void list_node_alloc_tail(m_list_info *p_info)
{
    prt(debug_list,"list alloc");
    pthread_mutex_lock(&p_info->list_lock);
    m_node *tmp=(m_node *)malloc(sizeof(m_node));
    tmp->data=malloc(p_info->data_size);
    memset(tmp->data,0,p_info->data_size);
    tmp->next=NULL;
    if(p_info->head!=NULL){
    //	info_p->tail->next=tmp->pre;
        tmp->pre=p_info->tail;
        p_info->tail->next=tmp;
        p_info->tail=p_info->tail->next;
    }else{
        tmp->pre=NULL;
        p_info->head=tmp;
    }
    p_info->tail=tmp;
    p_info->number++;

    prt(debug_list,"list alloc done");
    pthread_mutex_unlock(&p_info->list_lock);
}
//static void list_node_free_tail(m_list_info *info_p)
//{
//	m_node *tmp=info_p->tail;
//	if(info_p->head!=NULL){
//		if (info_p->tail->pre != NULL) {
//			info_p->tail = info_p->tail->pre;
//			info_p->tail->next = NULL;
//		}
//		free(tmp->data);
//		free(tmp);
//		info_p->number--;
//	}else{
//	}
//}
/* Function Description
 * name:
 * return:???????0? ????????????
 * args:
 * comment:??¦Ë??????????¦Ë??
 * todo:
 */
extern int list_node_seek(m_list_info *p_info,void *data)
{
    prt(debug_list,"list seek");
    pthread_mutex_lock(&p_info->list_lock);
    int ret=-1;
    p_info->cur=p_info->head;
    while(p_info->cur!=NULL)
    {
        ret=p_info->data_match_function(p_info->cur->data,data);
        if (ret >= 0) {
            pthread_mutex_unlock(&p_info->list_lock);
            prt(debug_list,"list seek done");
            return ret;
        } else
        {
            p_info->cur = p_info->cur->next;
            prt(debug_list,"checking next");
        }
    }
    //info_p->cur=info_p->tail;
    pthread_mutex_unlock(&p_info->list_lock);
    prt(debug_list,"list seek done");
    return -1;
}
extern int list_node_del_cur(m_list_info *p_info)
{
//	return 0;
//	prt(info,"list del");
    prt(debug_list,"list del  ,head %p, tail %p,cur %p",p_info->head,p_info->tail,p_info->cur);
    pthread_mutex_lock(&p_info->list_lock);
    if(p_info->number<=0||p_info->cur==NULL){
        pthread_mutex_unlock(&p_info->list_lock);
        return 1;
    }
    if (p_info->cur == p_info->tail) {//on tail
//		list_node_free_tail(p_info);
        m_node *tmp=p_info->tail;
//		if(p_info->head!=NULL){
        if(p_info->number>0){
            if (p_info->tail->pre != NULL) {//means number >1
                p_info->tail = p_info->tail->pre;// now tail change to former node
                p_info->tail->next = NULL;
            }else{//only one node
                p_info->head=NULL;
                p_info->tail=NULL;
            }
            free(tmp->data);
            free(tmp);
            p_info->number--;
        }else{
            prt(debug_list,"cant del cuz list is empty , total num %d",p_info->number);
        }
        prt(debug_list,"list del done,head %p, tail %p,cur %p",p_info->head,p_info->tail,p_info->cur);
        pthread_mutex_unlock(&p_info->list_lock);
        return 0;
    }
    if (p_info->cur == p_info->head) {// on head
        m_node *tmp=p_info->head;
        if(p_info->head->next!=NULL){
            p_info->head = p_info->head->next;
            p_info->head->pre = NULL;
        }else{
            p_info->head=NULL;
            p_info->tail=NULL;
        }
        free(tmp->data);
        free(tmp);
        p_info->number--;
        prt(debug_list,"list del done,head %p",p_info->head);
        pthread_mutex_unlock(&p_info->list_lock);
        return 0;
    }
// on middle
    //m_node *tmp=p_info->cur;
    p_info->cur->pre->next = p_info->cur->next;
    p_info->cur->next->pre = p_info->cur->pre;
    free(p_info->cur->data);
    free(p_info->cur);
    p_info->cur=NULL;
    //p_info->cur=p_info->tail;
    p_info->number--;
    prt(debug_list,"list del done,head %p",p_info->head);
    pthread_mutex_unlock(&p_info->list_lock);
    return 0;
}
extern void * list_get_current_data(m_list_info *info_p)
{
    return info_p->cur->data;
}
void list_overwirte_current_data(m_list_info *info_p,void *data)
{
    memcpy(info_p->cur->data,data,info_p->data_size);
}
/* Function Description
 * name:
 * return:
 * args:func=????????????  arg=????????
 * comment:?????????????
 * todo:arg§Õ???????? ??????????????????
 */
extern int list_operate_node_all(m_list_info *p_info,p_func func,void *arg)
{
//	prt(info,"list op ");
    pthread_mutex_lock(&p_info->list_lock);
//	prt(info,"list op  begin");
    m_node *tmp=p_info->head;
    while(tmp!=NULL)
    {
    // 	prt(info,"all");
        func(tmp->data,arg);
        tmp=tmp->next;
    }
    //info_p->cur=info_p->tail;
//	prt(info,"list op done");
    pthread_mutex_unlock(&p_info->list_lock);
    return 0;
}

void SetupSignalAction(int signo,void (*SignalAction)(int,siginfo_t*,void*))
{
    #if 1
    struct sigaction act;
    sigemptyset(&act.sa_mask);

    act.sa_flags=SA_SIGINFO;
    act.sa_sigaction = SignalAction;
    sigaction(signo, &act, NULL);
    #endif
}
void SignalAction_Trace(int signo,   siginfo_t *info,void* p)
{
    printf("|---->>>>get sig %d\n",signo);fflush(NULL);
    if (signo == SIGSEGV) {
        prt(info," err================================>seg fault,restart system\n");
    //    system("reboot");
        //  	system("pkill -9 zenith");
        exit(1);
    }

    #if 0
    void *array[10];
    size_t size;
    char **strings;
    size_t i, j;
    char link[32];
    char linkto[128]={0};

    if(signo==SIGPIPE){
        Log0("|---->>>> ##################\n");
        Log0("|---->>>> #####SIGPIPE######\n");
        Log0("|---->>>> ##################\n");
        return ;
    }

    mCommand head;
    memset(&head, 0, sizeof(mCommand));
    head.version = VERSION;
    head.prottype = PROTTYPE;
    head.objtype = htons(FORKEXIT);
    SendDataByClient((char*)&head, sizeof(mCommand), "127.0.0.1", 8888);

    size = backtrace(array, 10);
    strings = (char **)backtrace_symbols(array, size);

    sprintf(link,"/proc/%d/exe", info->si_pid);
    readlink(link,linkto, 128);
    linkto[127]=0;

        signo,  linkto, info->si_pid);
    for(i = 0; i < size; i++){
        Log0("%d %s\n",i,strings[i]);
    }


    sleep(1);
    free (strings);
    _exit(1);
    #endif
}
void exitTrace()
{
    prt(info,"calling exit@@@@@@@@@@@@@@@@@@@@");
    fflush(NULL);
}
extern int setup_sig()
{
    atexit(exitTrace);
    SetupSignalAction(SIGSEGV, SignalAction_Trace);//11
//	SetupSignalAction(SIGALRM, SignalAction_Trace);//14
//	SetupSignalAction(SIGTERM, SignalAction_Trace);//15
//	SetupSignalAction(SIGPWR , SignalAction_Trace);//30
//	SetupSignalAction(SIGKILL , SignalAction_Trace);
    SetupSignalAction(SIGPIPE , SignalAction_Trace);//30

    return 0;
}
