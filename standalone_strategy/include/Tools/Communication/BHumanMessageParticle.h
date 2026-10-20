#pragma once

struct BHumanMessage;

struct BHumanMessageParticle
{
  virtual ~BHumanMessageParticle() = default;
  virtual void operator>>(BHumanMessage&) const {}
  virtual void operator<<(const BHumanMessage&) {}
};

template<typename Message>
struct BHumanCompressedMessageParticle : public BHumanMessageParticle
{
};
