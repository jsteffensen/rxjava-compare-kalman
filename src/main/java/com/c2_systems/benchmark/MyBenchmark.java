package com.c2_systems.benchmark;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

import org.openjdk.jmh.annotations.*;
import org.openjdk.jmh.infra.Blackhole;
import org.reactivestreams.Publisher;

import io.reactivex.*;
import io.reactivex.flowables.GroupedFlowable;
import io.reactivex.functions.Function;
import io.reactivex.schedulers.Schedulers;

@BenchmarkMode(Mode.Throughput)
@Warmup(iterations = 5)
@Measurement(iterations = 5, time = 1, timeUnit = TimeUnit.SECONDS)
@Fork(value = 1,jvmArgsAppend = { "-XX:MaxInlineLevel=20" })
@OutputTimeUnit(TimeUnit.SECONDS)
@State(Scope.Thread)
public class MyBenchmark implements Function<Integer, Integer> {

    @Param({"10000"})
    public int count;

    @Param({"100"})
    public int compute;

    @Param({"3"})
    public int parallelism;


    Flowable<Integer> flatMap;
    Flowable<Integer> groupBy;
    Flowable<Integer> parallel;

    @Override
    public Integer apply(Integer t) throws Exception {
        Blackhole.consumeCPU(compute);
        return t;
    }

    @Setup
    public void setup() {

        final int cpu = parallelism;

        Integer[] ints = new Integer[count];
        Arrays.fill(ints, 777);

        Flowable<Integer> source = Flowable.fromArray(ints);

        flatMap = source.flatMap(new Function<Integer, Publisher<Integer>>() {
            @Override
            public Publisher<Integer> apply(Integer v) throws Exception {
                return Flowable.just(v).subscribeOn(Schedulers.computation())
                        .map(MyBenchmark.this);
            }
        }, cpu);

        groupBy = source.groupBy(new Function<Integer, Integer>() {

            int i;
            @Override
            public Integer apply(Integer v) throws Exception {
                return (i++) % cpu;
            }
        }).flatMap(new Function<GroupedFlowable<Integer, Integer>, Publisher<Integer>>() {
            @Override
            public Publisher<Integer> apply(GroupedFlowable<Integer, Integer> g) throws Exception {
                return g.observeOn(Schedulers.computation()).map(MyBenchmark.this);
            }
        });

        parallel = source.parallel(cpu).runOn(Schedulers.computation()).map(this).sequential();
    }

    void subscribe(Flowable<Integer> f, Blackhole bh) {
        PerfAsyncConsumer consumer = new PerfAsyncConsumer(bh);
        f.subscribe(consumer);
        consumer.await(count);
    }

    @Benchmark
    public void flatMap(Blackhole bh) {
        subscribe(flatMap, bh);
    }

    @Benchmark
    public void other(Blackhole bh) {
    	for(int i = 0; i<10000; i++) {
			Blackhole.consumeCPU(compute);
		}
    }

    @Benchmark
    public void groupBy(Blackhole bh) {
        subscribe(groupBy, bh);
    }

    @Benchmark
    public void parallel(Blackhole bh) {
        subscribe(parallel, bh);
    }
}